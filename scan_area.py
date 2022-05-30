#!/usr/bin/env python
from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Quaternion, Point, PointStamped
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import rospy
import moveit_commander
import sys
import os
import tf
from tf.transformations import quaternion_from_euler
from beach_cleanig_robot_sim.srv import SetMoveBasePose
from pp_one import pick_place, move_group_interface, garbage
from pp_multi import move_base, delete_garbage

class scan_area():
    def __init__(self):
        #### areas ################################################
        self.sight_area   = np.array([2.0, 1.0])      # [forward, sides] in [m]
        self.work_area    = np.array([7.0, 6.2])      # [length, width] - total dimensions of work area in [m]
        self.work_area[1] = np.floor(self.work_area[1] / self.sight_area[1])  # match the width with side sight
        
        #### squares - [num rows, num colums] #####################
        self.num_squares = np.array([np.ceil(self.work_area[0]/self.sight_area[0]), np.ceil(self.work_area[1]/self.sight_area[1]/2.0)])
        self.curr_square = [0, 0]  # start in bottom right corner of work area

        #### husky ################################################
        self.start_yaw = 15.0 * np.pi / 180.0  # [rad]
        self.rot_base2map = np.array([[np.cos(self.start_yaw), -np.sin(self.start_yaw)],
                                     [np.sin(self.start_yaw), np.cos(self.start_yaw)]])  # matrix transform from base_link to map
        
        self.start_xy_map  = np.array([0.0, 2.0])  # start point in /map frame
        self.start_xy_base = np.matmul(np.linalg.inv(self.rot_base2map), self.start_xy_map)
        self.husky_distance = 1.3                  # =0.65*2, the primary distance from the garbage that the husky will be scheduled to arrive
        self.markers()                             # draw area and calculate self.scan_poses

        self.unregister = False  # for callback function
        try:                     # start subscriber for callback function
            rospy.sleep(0.8)     # instead spin()                        
            rospy.Subscriber('gazebo/model_states', ModelStates, self.callback)  # (topic, massage, func)
            rospy.sleep(0.1)     # keeps node from exiting until the node has been shutdown
        except rospy.ROSInitException as e:
            rospy.loginfo("ModelStates massage call failed:  {0}".format(e))   
        
        #### garbages ################################################
        self.tf = tf.TransformListener()  # for transform /map to /base_link
        self.collected_names = []  # storage list (appending to NumPy arrays is catastrophically slower than appending to ordinary lists)

    def callback(self, data):  # retrieve the current loacation of the husky
        if not rospy.is_shutdown():
            self.curr_husky_pose = data.pose[data.name.index('husky')]
            self.curr_husky_xy = np.array([self.curr_husky_pose.position.x, self.curr_husky_pose.position.y])
            if not self.unregister:
                try:  # publish the markers
                    self.marker_array_publisher.publish(self.marker_array_msg)
                    self.marker_publisher.publish(self.line_strip)
                    self.marker_publisher2.publish(self.line_strip2)
                except rospy.ROSInitException:
                    pass

    def send_husky2start_pose(self):  # send husky to start pose
        NavToPoint = rospy.ServiceProxy('NavToGarbgLoc', SetMoveBasePose)
        goal = Pose()
        goal.position.x, goal.position.y, goal.position.z = self.start_xy_map[0], self.start_xy_map[1], 0.132266
        orientation_q = quaternion_from_euler(0.0, 0.0, self.start_yaw, 'ryxz')   # roll=pitch=0 on the plane
        goal.orientation = Quaternion(*orientation_q)
        NavToPoint(goal)  # sends the goal to the action server
        rospy.sleep(1.0)
        return NavToPoint(goal).result

    def markers(self):  # draw work area, sight area and scan arrows along the work area       
        self.marker_array_publisher = rospy.Publisher("/marker_array", MarkerArray, queue_size=1)
        self.marker_array_msg = MarkerArray()
        self.scan_poses = []
        for row in np.arange(self.num_squares[0]):
            for colum in np.arange(self.num_squares[1]):
                marker = Marker()
                marker.id = 0
                marker.ns = str([row, colum])
                marker.action = Marker.DELETE
                marker.action = Marker.ADD
                marker.header.frame_id = '/map'
                marker.type = 0
                marker.scale.x, marker.scale.y, marker.scale.z = 0.5, 0.2, 0.2
                marker.color.r, marker.color.g, marker.color.b , marker.color.a = 0.0, 0.0, 1.0, 1.0
                if (row == self.num_squares[0]-1 or row == 0) and (colum == self.num_squares[1]-1 or colum == 0):
                    marker.color.r, marker.color.g, marker.color.b , marker.color.a = 0.0, 1.0, 1.0, 1.0
                
                marker.pose = Pose()
                marker.pose.position.z = 0
                if colum < self.num_squares[1]-1:
                    marker.pose.position.y = self.start_xy_base[1] + 2*self.sight_area[1]*colum
                else:
                    marker.pose.position.y = self.start_xy_base[1] + self.work_area[1] - 2*self.sight_area[1]

                if colum % 2 == 0:  # even colum
                    orientation_q = quaternion_from_euler(0.0, 0.0, self.start_yaw, 'ryxz')
                    if row < self.num_squares[0]-1:
                        marker.pose.position.x = self.start_xy_base[0] + self.sight_area[0]*row
                    else:
                        marker.pose.position.x = self.start_xy_base[0] + self.work_area[0] - self.sight_area[0]
                else:
                    orientation_q = quaternion_from_euler(0.0, 0.0, self.start_yaw + np.pi, 'ryxz')
                    if row < self.num_squares[0]-1:
                        marker.pose.position.x = self.start_xy_base[0] + self.work_area[0] - self.sight_area[0]*row
                    else:
                        marker.pose.position.x = self.start_xy_base[0] + self.sight_area[0]
                marker.pose.orientation = Quaternion(*orientation_q) 
               
                # transform to map frame:
                marker_xy = np.matmul(self.rot_base2map, np.array([marker.pose.position.x, marker.pose.position.y]))
                marker.pose.position.x = marker_xy[0]
                marker.pose.position.y = marker_xy[1]
                
                self.scan_poses.append(marker.pose)
                self.marker_array_msg.markers.append(marker)

        self.scan_poses = np.array(self.scan_poses)
        self.scan_poses = self.scan_poses.reshape((int(self.num_squares[0]), int(self.num_squares[1])))
        
        #### squares of area #########################################################################################
        self.marker_publisher, self.marker_publisher2 = rospy.Publisher("/marker", Marker, queue_size=1), rospy.Publisher("/marker2", Marker, queue_size=1)
        self.line_strip, self.line_strip2 = Marker(), Marker()
        self.line_strip.id, self.line_strip2.id = 1, 2
        self.line_strip.action = Marker.DELETEALL; self.line_strip2.action = Marker.DELETEALL
        self.line_strip.action = Marker.ADD; self.line_strip2.action = Marker.ADD
        self.line_strip.ns, self.line_strip2.ns = "work area", "sight area"
        self.line_strip.header.frame_id, self.line_strip2.header.frame_id = '/map', '/base_link'
        self.line_strip.type = self.line_strip2.type = 4
        self.line_strip.color.r, self.line_strip.color.g, self.line_strip.color.b, self.line_strip.color.a = 1.0, 0.647, 0.0, 1.0
        self.line_strip2.color.r, self.line_strip2.color.g, self.line_strip2.color.b, self.line_strip2.color.a = 0.588, 0.294, 0.0, 1.0
        self.line_strip.pose.orientation.x = self.line_strip.pose.orientation.y = self.line_strip.pose.orientation.z = 0.0
        self.line_strip2.pose.orientation.x = self.line_strip2.pose.orientation.y = self.line_strip2.pose.orientation.z = 0.0
        self.line_strip.pose.orientation.w = self.line_strip2.pose.orientation.w = 1.0
        self.line_strip.scale.x = self.line_strip2.scale.x = 0.15
       
        points_work_base = self.start_xy_base + np.array([[0.0, -self.sight_area[1]],
                                                [self.work_area[0], -self.sight_area[1]],
                                                [self.work_area[0], -self.sight_area[1] + self.work_area[1]],
                                                [0.0, -self.sight_area[1] + self.work_area[1]]])
        points_work_map = np.array([np.matmul(self.rot_base2map, points_work_base[i]) for i in range(4)])
        
        points_sight = np.array([[0.0, self.sight_area[1]],
                                [self.sight_area[0], self.sight_area[1]],
                                [self.sight_area[0], -self.sight_area[1]],
                                [0.0, -self.sight_area[1]]])
        for i in range(5):
            p1, p2 = Point(), Point()
            if i != 4:
                p1.x, p1.y = points_work_map[i, 0], points_work_map[i, 1]
                p2.x, p2.y = points_sight[i, 0], points_sight[i, 1]
            else:   # to close the square
                p1.x, p1.y = points_work_map[0, 0], points_work_map[0, 1]
                p2.x, p2.y = points_sight[0, 0], points_sight[0, 1]
            self.line_strip.points.append(p1)
            self.line_strip2.points.append(p2)

    def update_sight_distances(self):  # after each updating of self.distances
        self.sight_distances = []
        for i in range(len(self.sight_points)):
            if(self.sight_garbages[i]._name in self.collected_names):
                self.sight_distances.append(np.inf)
            else:
                self.sight_distances.append(np.linalg.norm(self.sight_points[i]))
        self.sight_distances = np.array(self.sight_distances)
        if len(self.sight_points) > 0:
            if np.min(self.sight_distances) != np.inf:
                print(u"\u001b[34mcurrent sight distances:"); print(np.round(self.sight_distances, 2))

    def go_scan(self, goal_pose):  # go to goal_pose
            NavToPoint = rospy.ServiceProxy('NavToGarbgLoc', SetMoveBasePose)        
            NavToPoint(goal_pose)      # sends the goal to the action server
            rospy.sleep(1.3)           # prevent skip scan square
            self.update_sight_distances()        
            return NavToPoint(goal_pose).result

    def continue_scan(self):  # call go_scan base on curr_square
        print('')
        if self.curr_square[0] < self.num_squares[0]-1:    # go forward
            print(u"\u001b[32mmove forward to next scan pose")
            self.curr_square += np.array([1, 0])
            res = self.go_scan(self.scan_poses[self.curr_square[0],self.curr_square[1]])
            return res
        elif self.curr_square[1] < self.num_squares[1]-1:  # rotate
            print(u"\u001b[32mrotate to next scan pose")
            self.curr_square = np.array([0, self.curr_square[1]+1])
            res = self.go_scan(self.scan_poses[self.curr_square[0],self.curr_square[1]])
            return res
        else:
            print(u"\u001b[32mThe scanning was done!")
            return False  # for ending the process

    def main(self):
            print(u"\u001b[36mwork area after offsetting width: " + str(self.work_area))
            print(u"\u001b[36mnum scan squares: " + str(self.num_squares))
            print(u"\u001b[36mstart xy: "  + str(self.start_xy_map))
            print(u"\u001b[36mstart yaw: " + str(180.0*self.start_yaw/np.pi) + " deg")
            # send the arm to home position, for avoid collision of the hand with the obstacle and improve the scanning
            moveit_commander.roscpp_initialize(sys.argv)  # init moveit_commander, allow to communicate with a move_group
            self.ur5_group = moveit_commander.MoveGroupCommander("ur5")  # interface to a planning group (of joints), can be used to plan and execute motions
            self.ur5_group.set_planner_id("RRT")          # choice RRT-path planning algorithm for the arm
            print(u"\u001b[32m---- step 0_1: send the arm to home pose ----")
            self.ur5_group.set_named_target("home")       # set the target position for the arm
            self.ur5_group.go(wait=True)                  # plan and execute the position
            self.ur5_group.stop()                         # ensures that there is no residual movement

            arrive_start_point = self.send_husky2start_pose()
            if arrive_start_point:
                while 1:  # until break
                    #### analyze sight area #######
                    self.sight_garbages, self.sight_points, self.sight_distances = [], [], []
                    try:  # service /gazebo/get_world_properties
                        world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
                        models_names = world_properties().model_names
                        for name in models_names:
                            if('obj' in name):  # only the garbage has a name that contains "obj"
                                gar = garbage(name)
                                point_global = PointStamped()
                                point_global.header.stamp = rospy.Time.now()
                                point_global.header.frame_id = '/map'  # map is the global frame
                                point_global.point.x, point_global.point.y, point_global.point.z = gar.garbage_point[0], gar.garbage_point[1], 0.0
                                gar.sub.unregister()  # unsubscriber to 'gazebo/model_states' topic
                                self.tf.waitForTransform("/map", "/base_link", rospy.Time.now(), rospy.Duration(4.0))
                                
                                point = self.tf.transformPoint('/base_link', point_global)
                                # check if the garbage in the sight area:
                                if (0 < point.point.x and point.point.x <= self.sight_area[0] and abs(point.point.y) <= self.sight_area[1]):
                                    self.sight_garbages.append(gar)
                                    self.sight_points.append(np.array([point.point.x, point.point.y]))
                                    self.sight_distances.append(np.linalg.norm(np.array([point.point.x, point.point.y])))
 
                        self.sight_garbages = np.array(self.sight_garbages)
                        self.sight_points = np.array(self.sight_points)
                        self.sight_distances = np.array(self.sight_distances)
                    except rospy.ServiceException as e:
                        rospy.loginfo("get_world_properties service call failed:  {0}".format(e))
                    
                    if(len(self.sight_points) > 0):
                        print(u"\u001b[35mthe garbage/s named " + str([n._name for n in self.sight_garbages]) + " is in current sight area!")
                    else:
                        print(u"\u001b[35mthere isn't garbage in current sight area")

                    #### collect the garbage/s in the current sight area #######
                    for g in range(len(self.sight_points)):
                        self.sorted_indicies = np.argsort(self.sight_distances)  # indicies that sorting the array from small to large
                        self.nearest_index = self.sorted_indicies[0]

                        ##### check if there are more than 1 nearest #####
                        if len(self.sight_points) >= 2:
                            if(abs(self.sight_distances[self.nearest_index] - self.sight_distances[self.sorted_indicies[1]]) <= 0.1):
                                print(u"\u001b[35mthere are 2 garbages in the same distance from the husky!")
                                if len(self.sight_points) == 2:  # check distances from the next center squre
                                    print(u"\u001b[35mcollect the more distant from next square")
                                    if self.curr_square[0] == self.num_squares[0]-1 and self.curr_square[1] == self.num_squares[1]-1:
                                        print(u"\u001b[32mThis the last square, so there isn't a preference for certain garbage from the two")
                                    else:
                                        next_square = self.scan_poses[self.curr_square[0]+1, self.curr_square[1]+1]
                                        next_square_xy = PointStamped()  # for transform to base_link frame
                                        next_square_xy.header.stamp = rospy.Time.now()
                                        next_square_xy.header.frame_id = '/map'  # map is the global frame
                                        next_square_xy.point.x, next_square_xy.point.y, point_global.point.z = next_square.position.x, next_square.position.y, 0.0
                                        self.tf.waitForTransform("/map", "/base_link", rospy.Time.now(), rospy.Duration(4.0))
                                        next_square_xy = self.tf.transformPoint('/base_link', next_square_xy)
                                        next_square_xy = np.array([next_square_xy.point.x, next_square_xy.point.y])

                                        next_square_distance0 = np.linalg.norm(self.sight_points[self.sorted_indicies[0]] - next_square_xy)
                                        next_square_distance1 = np.linalg.norm(self.sight_points[self.sorted_indicies[1]] - next_square_xy)
                                        if next_square_distance1 > next_square_distance0:  # collect first the more far garbage
                                            self.nearest_index = self.sorted_indicies[1]                                
                                else:  # check relative distances from the garbage that nearest third
                                    print(u"\u001b[35mcollect the more distant garbage from third nearest garbage")
                                    rel_distance0 = np.linalg.norm(self.sight_points[self.sorted_indicies[0]] - self.sight_points[self.sorted_indicies[2]])
                                    rel_distance1 = np.linalg.norm(self.sight_points[self.sorted_indicies[1]] - self.sight_points[self.sorted_indicies[2]])
                                    if rel_distance1 > rel_distance0:  # collect first the more far garbage
                                        self.nearest_index = self.sorted_indicies[1]
                    
                        print(u"\u001b[35mgarbage to collect now: " + self.sight_garbages[self.nearest_index]._name)

                        #### move_group with the garbage to collect now ################################################
                        move_group = move_group_interface(self.sight_garbages[self.nearest_index])  # this class from pp_one.py
                        pp_process = pick_place(move_group.ur5_group, move_group.gripper_group, self.sight_garbages[self.nearest_index])  # this class from pp_one.py
                        pp_process.step0()  # send the arm to home position, for avoid collision of the hand with the garbage
                        
                        move_base(self.nearest_index, pp_process, self.sight_distances, self.husky_distance)
                        self.collected_names.append(self.sight_garbages[self.nearest_index]._name)
                        delete_garbage(self.nearest_index, self.sight_garbages)
                        self.update_sight_distances()
                        
                        move_group.ur5_group.clear_pose_targets(); move_group.gripper_group.clear_pose_targets()
                        if g < (len(self.sight_points) - 1):
                            print(u"\u001b[32m---------- next garbage ----------")
                        else:
                            print(u"\u001b[32mThe robot has finished in this square!")

                    #### move the robot #########################
                    result4 = self.continue_scan() 
                    if result4:
                        rospy.loginfo("goal execution done!")
                    else:
                        break
            else: 
                print(u"\u001b[31mThe husky wasn't succeeded arrrive to start point of the scanning")
            self.unregister = True
            rospy.sleep(0.4)  # to ensure stopping the publishing before unregister 
            self.marker_array_publisher.unregister()
            self.marker_publisher.unregister()
            self.marker_publisher2.unregister()

if __name__ == '__main__':  # if the python node is executed as main process (sourced directly)
    rospy.init_node('scan_area_node', anonymous=True)  # init a node to let publish and subscribe
    scan = scan_area()
    try:
        scan.main()
        #### ending #################################################
        rospy.sleep(2)
        rospy.signal_shutdown("ctrl+c was typed")
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInitException or rospy.ROSException:
        pass
