#!/usr/bin/env python
from gazebo_msgs.srv import GetWorldProperties, DeleteModel
from gazebo_msgs.msg import ModelStates
import numpy as np
import rospy
import moveit_commander
from pp_one import pick_place, move_group_interface, garbage

def move_base(nearest_i, pp_i, distances, husky_distance):  # send husky gradually close to the garbage
    if distances[nearest_i] > husky_distance:
        result = pp_i.send_husky(husky_distance, pp_i.curr_xy_base_link, pp_i._object, pp_i.husky_target_xy)  # send husky close to the garbage
        if result:
            rospy.loginfo("goal execution done!")
            result2 = pp_i.send_husky(husky_distance/2.0, pp_i.curr_xy_base_link, pp_i._object, pp_i.husky_target_xy)  # send husky closer for picking
            if result2:
                rospy.loginfo("goal execution done!")
    else:
        result3 = pp_i.send_husky(husky_distance/2.0, pp_i.curr_xy_base_link, pp_i._object, pp_i.husky_target_xy)  # send husky closer for picking
        if result3:
            rospy.loginfo("goal execution done!")

def delete_garbage(i, garbages):  # delete garbage from gazebo
    rospy.sleep(1.0)  # for look the final position of the robot relative to this garbage
    try:  # service /gazebo/delete_model for delete the current collected garbage
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model(garbages[i]._name)
        print(u"\u001b[35mthe garbage that named " + garbages[i]._name + " was deleted")
    except rospy.ServiceException as e:
        rospy.loginfo("Delete Model service call failed:  {0}".format(e)) 

class multi_pp():
    def __init__(self):
        #### husky ################################################
        self.husky_distance = 1.3  # =0.65*2, the primary distance from the garbage that the husky will be scheduled to arrive
        try:                       # start subscriber for 'callback' function
            rospy.sleep(0.8)       # instead spin()                        
            rospy.Subscriber('gazebo/model_states', ModelStates, self.callback)  # (topic, massage, func)
            rospy.sleep(0.1)       # keeps node from exiting until the node has been shutdown
        except rospy.ROSInitException as e:
            rospy.loginfo("ModelStates massage call failed:  {0}".format(e))   
        
        #### garbages ################################################
        self.n = 0  # number of garbages in gazebo
        self.garbages, self.points, self.distances, self.collected_indicies = [], [], [], []  # storage lists
        try:  # service /gazebo/get_world_properties for calculate n
            self.world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            self.models_names = self.world_properties().model_names
            for name in self.models_names:
                if('obj' in name):  # only the garbages have names that contains "obj"
                    gar = garbage(name)
                    self.n += 1
                    self.garbages.append(gar)
                    self.points.append(gar.garbage_point)
                    self.distances.append(np.linalg.norm(gar.garbage_point - self.curr_husky_xy))
                    gar.sub.unregister()  # unsubscriber to 'gazebo/model_states' topic 

            self.garbages = np.array(self.garbages)
            self.points = np.array(self.points)
            self.distances = np.array(self.distances)
        except rospy.ServiceException as e:
            rospy.loginfo("get_world_properties service call failed:  {0}".format(e))             
        if len(self.points) > 0:
            print(u"\u001b[34mpoints:");    print(np.round(self.points, 2))
            print(u"\u001b[34mcurrent distances:"); print(np.round(self.distances, 2))

    def callback(self, data): # retrieve the current loacation of the husky, for calculate the nearest garbage
        if(not rospy.is_shutdown()):
            self.curr_husky_pose = data.pose[data.name.index('husky')]
            self.curr_husky_xy = np.array([self.curr_husky_pose.position.x, self.curr_husky_pose.position.y])

    def initialize_distances(self):  # initialize storage list distances
        self.distances = []
        for i in range(self.n):  # update the list base on the new location of the husky
            if i in self.collected_indicies:  # after place nearest garbage in the tin - covert nearest distance to infinity for find the new current min distance
                self.distances.append(np.inf)
            else:
                self.distances.append(np.linalg.norm(self.points[i] - self.curr_husky_xy))
        self.distances = np.array(self.distances)
        if len(self.distances) > 0:
            print(u"\u001b[34mcurrent distances:"); print(np.round(self.distances, 2))

    def main(self):
        for g in range(self.n):
            self.nearest_index = np.argmin(self.distances)
            ##### check if there are more than 1 nearest #####
            if self.n >= 3:
                self.sorted_indicies = np.argsort(self.distances)  # sorting from small to large
                self.nearest_index = self.sorted_indicies[0]
                if(abs(self.distances[self.sorted_indicies[0]] - self.distances[self.sorted_indicies[1]]) <= 0.1):
                    print(u"\u001b[37mthere are 2 garbages in the same distance from the husky!")
                    print(u"\u001b[37mcollect the more distant garbage first")
                    # check relative distances from the garbage that nearest third     
                    rel_distances0 = np.linalg.norm(self.points[self.sorted_indicies[0]] - self.points[self.sorted_indicies[2]])
                    rel_distances1 = np.linalg.norm(self.points[self.sorted_indicies[1]] - self.points[self.sorted_indicies[2]])
                    if rel_distances1 > rel_distances0:  # collect first the more far garbage
                        self.nearest_index = self.sorted_indicies[1]

            ############ move_group with the correct nearest_index ###############################
            move_group = move_group_interface(self.garbages[self.nearest_index])  # this class from pp_one.py
            pp_process = pick_place(move_group.ur5_group, move_group.gripper_group, self.garbages[self.nearest_index])  # this class from pp_one.py
            pp_process.step0()  # send the arm to home position, for avoid collision of the hand with the obstacle
            
            move_base(self.nearest_index, pp_process, self.distances, self.husky_distance)
            self.collected_indicies.append(self.nearest_index)
            delete_garbage(self.nearest_index, self.garbages) 
            self.initialize_distances()            
            move_group.ur5_group.clear_pose_targets(); move_group.gripper_group.clear_pose_targets()

            if g < (self.n - 1):
                print(u"\u001b[32m---------- next garbage ----------")
            else:
                print(u"\u001b[32mThe robot has finished!")

if __name__ == '__main__':  # if the python node is executed as main process (sourced directly)
    rospy.init_node('pp_multi_node', anonymous=True)  # init a node to let publish and subscribe
    multi = multi_pp()
    if multi.n > 0:
        multi.main()
        #### ending ##########################    
        rospy.sleep(2)
        rospy.signal_shutdown("ctrl+c was typed")
        moveit_commander.roscpp_shutdown()
    else:
        print(u"\u001b[32mNo garbage to collect!")