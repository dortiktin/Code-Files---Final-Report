#!/usr/bin/env python
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Twist
from math import sqrt, atan2
import numpy as np
import sys
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list  # return [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt
from beach_cleanig_robot_sim.srv import SetMoveBasePose
import pickle
from tabulate import tabulate
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

class garbage:
    def __init__(self, name):  # defaul relative entity is map frame
        self._name = name
        #self.H, self.R = 0.199984, 0.0339844  # unit1_cylinder height and radius, [m]
        self.H, self.R = 0.33097, 0.0362935  # unit2_cylinder height and radius, [m]
        try:  # start subscriber for 'callback' function
            rospy.sleep(0.8)  # instead spin(), in order to continue the code                        
            self.sub = rospy.Subscriber('gazebo/model_states', ModelStates, self.callback)  # (topic, massage, func)
            rospy.sleep(0.1)  # keeps node from exiting until the node has been shutdown
        except rospy.ROSInitException as e:
            rospy.loginfo("ModelStates massage call failed:  {0}".format(e))

    def callback(self, data):
        if(not rospy.is_shutdown()):
            self.garbage_pose = data.pose[data.name.index(self._name)]
            self.garbage_point = np.array([self.garbage_pose.position.x, self.garbage_pose.position.y])  # center of garbage on xy plane
            self.orientation = np.array([self.garbage_pose.orientation.x, self.garbage_pose.orientation.y, self.garbage_pose.orientation.z, self.garbage_pose.orientation.w])

class pick_place:  # by steps
    def __init__(self, ur5_group, gripper_group, object):
        self.ur5 = ur5_group; self.gripper = gripper_group
        self._object = object
        self.pose_pick = Pose(); self.pose_place  = Pose()
        self.option = 0; self.husky_target_xy = [0.0, 0.0]
        
        try:  # start subscriber for 'callback' function
            rospy.sleep(0.8)  # instead spin()                        
            rospy.Subscriber('gazebo/model_states', ModelStates, self.callback)  # (topic, massage, func)
            rospy.sleep(0.1)  # keeps node from exiting until the node has been shutdown
        except rospy.ROSInitException as e:
            rospy.loginfo("ModelStates massage call failed:  {0}".format(e))   

    def callback(self, data):  # retrieve the current loacation of the husky
        if(not rospy.is_shutdown()):
            self.curr_husky_pose = data.pose[data.name.index('husky')]
            self.curr_xy_base_link = np.array([self.curr_husky_pose.position.x, self.curr_husky_pose.position.y])

    def all_close(self, goal, actual, tolerance=0.03):  # testing if a list of values are within a tolerance of their counterparts in another list
        ''' @param: goal   A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool '''
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False
        elif type(goal) is PoseStamped:
            return self.all_close(pose_to_list(goal.pose), pose_to_list(actual.pose))
        elif type(goal) is Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual))
        return True   

    def step0(self):  # send the arm to home position
        print(u"\u001b[32m---- step 0_1: send the arm to home pose ----")
        self.ur5.set_named_target("home")  # set the target position for the arm
        self.ur5.go(wait=True)  # plan and execute the position
        self.ur5.stop()  # ensures that there is no residual movement
        cur_joints = self.ur5.get_current_joint_values()
        home_joints = [0.000, -1.972, 2.129, -2.112, 0.105, 0.105]
        return self.all_close(home_joints, cur_joints)  # for testing

    def send_husky(self, d, curr_xy_base_link, _object, husky_target_xy):  # d - the distance along the connection vector
        if d > 1.0:
            print(u"\u001b[32m---- step 0_2: send husky close to the garbage ----")
        else:
            print(u"\u001b[32m---- step 0_3: send husky closer ----")
        #### calculate the position ################################################
        m = (curr_xy_base_link[1] - _object.garbage_point[1]) / (curr_xy_base_link[0] - _object.garbage_point[0])
        p1 = np.array([_object.garbage_point[0] + (d/sqrt(m**2+1)), _object.garbage_point[1] + (m*d/sqrt(m**2+1))])
        p2 = np.array([_object.garbage_point[0] - (d/sqrt(m**2+1)), _object.garbage_point[1] - (m*d/sqrt(m**2+1))])
        d1 = np.linalg.norm(curr_xy_base_link - p1)
        d2 = np.linalg.norm(curr_xy_base_link - p2)
        '''print("p1: " + str(p1)) print("p2: " + str(p2)) print("base_link: " + str(curr_xy_base_link)) print("d1: " + str(d1)) print("d2: " + str(d2))'''
        if(d1 < d2):
            husky_target_xy = p1
        else:
            husky_target_xy = p2

        NavToPoint = rospy.ServiceProxy('NavToGarbgLoc', SetMoveBasePose)
        goal = Pose()
        goal.position.x, goal.position.y = husky_target_xy[0], husky_target_xy[1]
        goal.position.z = 0.132266  # the z value of base_link frame
        
        #### calculate the orientation ################################################
        u = np.array([_object.garbage_point[0] - husky_target_xy[0], _object.garbage_point[1] - husky_target_xy[1]])
        yaw = atan2(u[1], u[0])  # atan2(y, x) = atan(y/x) - returns in [rad]
        orientation_q = quaternion_from_euler(0.0, 0.0, yaw, 'ryxz')   # roll=pitch=0 on the plane
        goal.orientation = Quaternion(*orientation_q)
        NavToPoint(goal) # sends the goal to the action server
        return NavToPoint(goal).result

    def move_husky_foward(self, dist):
        print(u"\u001b[32m---- step 0_3: send husky closer ----")
        pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        msg = Twist()
        msg.linear.x = dist  # m/s
        now = rospy.Time.now(); rate = rospy.Rate(7)  # Save current time and set publish rate
        # for the next 1 seconds publish cmd_vel move commands to husky
        while rospy.Time.now() < now + rospy.Duration.from_sec(1.0):
            pub.publish(msg)
            rate.sleep() 

    ### pick process ################################################
    def step1_1(self):  # open the gripper
        print(u"\u001b[32m---- step 1_1: open the gripper ----")
        self.gripper.set_named_target("open")  # set the target position for the gripper
        self.gripper.go(wait=True)  # plan and execute the position
        self.gripper.stop()  # ensures that there is no residual movement

    def step1_2(self):  # bring the arm above the garbage
        for i in range(3):
            print(u"\u001b[32m---- step 1_2: bring the arm above the garbage ----")
            self.option = 1
            self.pose_pick.position.x = self._object.garbage_point[0] - self._object.R - 0.3
            self.pose_pick.position.y = self._object.garbage_point[1]  # close to the garbage
            self.pose_pick.position.z = self._object.H - i/10.0  # garbage z center, for vertical garbage
            '''print('___________________________________')
            print("garbage_garbage_point: " + str([round(num, 3) for num in self._object.garbage_point]))
            print("husky_target_xy: " + str([round(nu, 3) for nu in self.husky_target_xy]))
            print("pose_pick: " + str([round(n, 3) for n in [self.pose_pick.position.x, self.pose_pick.position.y, self.pose_pick.position.z]]))
            print("xyz norm:  " + str(round(np.linalg.norm([self.pose_pick.position.x-self.husky_target_xy[0], self.pose_pick.position.y-self.husky_target_xy[1], self.pose_pick.position.z]),3)))
            print("xy norm:   " + str(round(np.linalg.norm([self.pose_pick.position.x-self.husky_target_xy[0], self.pose_pick.position.y-self.husky_target_xy[1]]), 3)))
            print('___________________________________')'''

            print(u"\u001b[34mRPY = (90, 0, 0)")  # RPY = (90, 0, 0) - for vertical garbage
            self.pose_pick.orientation.x, self.pose_pick.orientation.y = sqrt(0.5), 0.0
            self.pose_pick.orientation.z, self.pose_pick.orientation.w = 0.0, sqrt(0.5)

            self.ur5.set_pose_target(self.pose_pick)  # set the target pose for the arm
            self.ur5.go(wait=True)  # plan and execute the position
            self.ur5.stop()  # ensures that there is no residual movement

            cur_pose = self.ur5.get_current_pose().pose
            
            if not self.all_close(self.pose_pick, cur_pose):  # check more points
                print(u"\u001b[34mRPY = (-90, 0, 0)")  # RPY = (-90, 0, 0) - for vertical garbage
                self.pose_pick.orientation.x, self.pose_pick.orientation.y = -sqrt(0.5), 0.0
                self.pose_pick.orientation.z, self.pose_pick.orientation.w = 0.0, sqrt(0.5)

                self.ur5.set_pose_target(self.pose_pick)  # set the target pose for the arm
                self.ur5.go(wait=True)  # plan and execute the position
                self.ur5.stop()  # ensures that there is no residual movement

                cur_pose = self.ur5.get_current_pose().pose
            else: break
            
            if not self.all_close(self.pose_pick, cur_pose):  # check more point
                print(u"\u001b[34mRPY = (90, 0, 90)")  # RPY = (90, 0, 90) - for vertical garbage
                self.option = 2
                self.pose_pick.position.x = self._object.garbage_point[0]  # close to the garbage
                self.pose_pick.position.y = self._object.garbage_point[1] - self._object.R - 0.3

                self.pose_pick.orientation.x, self.pose_pick.orientation.y = 0.5, 0.5
                self.pose_pick.orientation.z, self.pose_pick.orientation.w = 0.5, 0.5

                self.ur5.set_pose_target(self.pose_pick)  # set the target pose for the arm
                self.ur5.go(wait=True)  # plan and execute the position
                self.ur5.stop()  # ensures that there is no residual movement
                
                cur_pose = self.ur5.get_current_pose().pose
            else: break

            if not self.all_close(self.pose_pick, cur_pose):  # check more point
                print(u"\u001b[34mRPY = (-90, 0, 90)")  # RPY = (-90, 0, 90) - for vertical garbage
                self.pose_pick.orientation.x, self.pose_pick.orientation.y = -0.5, -0.5
                self.pose_pick.orientation.z, self.pose_pick.orientation.w = 0.5, 0.5

                self.ur5.set_pose_target(self.pose_pick)  # set the target pose for the arm
                self.ur5.go(wait=True)  # plan and execute the position
                self.ur5.stop()  # ensures that there is no residual movement
                
                cur_pose = self.ur5.get_current_pose().pose
            else: break

            if not self.all_close(self.pose_pick, cur_pose):  # check more point
                print(u"\u001b[34mRPY = (90, 0, -90)")  # RPY = (90, 0, -90) - for vertical garbage
                #self.option = 3
                self.pose_pick.position.x = self._object.garbage_point[0]  # close to the garbage
                self.pose_pick.position.y = self._object.garbage_point[1] + self._object.R + 0.3

                self.pose_pick.orientation.x, self.pose_pick.orientation.y = 0.5, -0.5
                self.pose_pick.orientation.z, self.pose_pick.orientation.w = -0.5, 0.5

                self.ur5.set_pose_target(self.pose_pick)  # set the target pose for the arm
                self.ur5.go(wait=True)  # plan and execute the position
                self.ur5.stop()  # ensures that there is no residual movement
                
                cur_pose = self.ur5.get_current_pose().pose
            else: break

            if not self.all_close(self.pose_pick, cur_pose):  # check more point
                print(u"\u001b[34mRPY = (-90, 0, -90)")  # RPY = (-90, 0, -90) - for vertical garbage
                self.pose_pick.orientation.x, self.pose_pick.orientation.y = -0.5, 0.5
                self.pose_pick.orientation.z, self.pose_pick.orientation.w = -0.5, 0.5

                self.ur5.set_pose_target(self.pose_pick)  # set the target pose for the arm
                self.ur5.go(wait=True)  # plan and execute the position
                self.ur5.stop()  # ensures that there is no residual movement
                
                cur_pose = self.ur5.get_current_pose().pose
            else: break

        return self.all_close(self.pose_pick, cur_pose)  # for testing

    def step1_3(self):  # bring the arm down
        print(u"\u001b[32m---- step 1_3: bring the arm down ----")
        self.pose_pick.position.z -= self._object.H / 1.7

        self.ur5.set_pose_target(self.pose_pick)  # set the target pose for the arm
        self.ur5.go(wait=True)  # plan and execute the position
        self.ur5.stop()  # ensures that there is no residual movement

        cur_pose = self.ur5.get_current_pose().pose
        return self.all_close(self.pose_pick, cur_pose)  # for testing

    def step1_4(self):  # bring the arm closer
        print(u"\u001b[32m---- step 1_4: bring the arm closer ----")
        if self.option == 1:
            self.pose_pick.position.x += 0.18
        elif self.option == 2:
            self.pose_pick.position.y += 0.18
        # self.pose_pick.position.z -= self._object.H/2
        self.ur5.set_pose_target(self.pose_pick)  # set the target pose for the arm
        self.ur5.go(wait=True)  # plan and execute the position
        self.ur5.stop()  # ensures that there is no residual movement

        cur_pose = self.ur5.get_current_pose().pose
        return self.all_close(self.pose_pick, cur_pose)  # for testing

    def step1_5(self):  # close the gripper
        print(u"\u001b[32m---- step 1_5: close the gripper ----")
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[0] = 14.0 * np.pi / 180.0    # for unit1_cylinder 
        self.gripper.go(joint_goal, wait=True)  # plan and execute the position
        self.gripper.stop()  # ensures that there is no residual movement
        
        current_joints = self.gripper.get_current_joint_values()
        return self.all_close(joint_goal, current_joints)  # for testing

    ### place process ################################################
    def step2_1(self):  # pick up the garbage
        print(u"\u001b[32m---- step 2_1: pick up the garbage ----")
        self.pose_pick.position.z += 0.1
        self.ur5.set_pose_target(self.pose_pick)  # set the target pose for the arm
        self.ur5.go(wait=True)  # plan and execute the position
        self.ur5.stop()   # ensures that there is no residual movement

        cur_pose = self.ur5.get_current_pose().pose
        return self.all_close(self.pose_pick, cur_pose)  # for testing

    def step2_2(self):  # place in the dedicated tin and clear groups poses
        print(u"\u001b[32m---- step 2_2: place in the tin ----")
        tin_position = np.concatenate(self.curr_xy_base_link + np.array([-0.03, 0.0]), np.array([0.388])) # position of the tin in 3D space, is fixed in relation to base_link
        self.pose_place.position.x = tin_position[0]
        self.pose_place.position.y = tin_position[1]
        self.pose_place.position.z = tin_position[2] + 0.2  # up to tin
        # RPY = (90, 0, 180):
        self.pose_place.orientation.x, self.pose_place.orientation.y = 0.0, sqrt(0.5)
        self.pose_place.orientation.z, self.pose_place.orientation.w = sqrt(0.5), 0.0
        self.ur5.set_pose_target(self.pose_place)  # set the target pose for the arm
        self.ur5.go(wait=True)  # plan and execute the position
        self.ur5.stop()  # ensures that there is no residual movement

        self.gripper.set_named_target("open")  # set the target position for the gripper
        self.gripper.go(wait=True)  # plan and execute the position
        self.gripper.stop()  # ensures that there is no residual movement
        cur_pose = self.ur5.get_current_pose().pose
        self.ur5.clear_pose_targets()
        self.gripper.clear_pose_targets()

        return self.all_close(self.pose_place, cur_pose)  # for testing

    def ompl_planner_analysis(self):  # analyze the ompl planner for the arm and gripper
        pose1 = Pose()
        pose1.position.x, pose1.position.y, pose1.position.z = 0.7, 0.0, 0.2
        # RPY = (90, 0, 0):
        pose1.orientation.x, pose1.orientation.y = sqrt(0.5), 0.0
        pose1.orientation.z, pose1.orientation.w = 0.0, sqrt(0.5)

        pose2 = Pose()
        pose2.position.x, pose2.position.y, pose2.position.z = 0.0, 0.5, 0.2
        # RPY = (90, 0, 90):
        pose2.orientation.x, pose2.orientation.y = 0.5, 0.5
        pose2.orientation.z, pose2.orientation.w = 0.5, 0.5

        pose3 = Pose()
        tin_position = [-0.03, 0.0, 0.388]  # position of the tin in 3D space, is fixed in relation to base_link
        pose3.position.x = tin_position[0]
        pose3.position.y = tin_position[1]
        pose3.position.z = tin_position[2] + 0.3  # up to tin
        # RPY = (90, 0, 180):
        pose3.orientation.x, pose3.orientation.y = 0.0, sqrt(0.5)
        pose3.orientation.z, pose3.orientation.w = sqrt(0.5), 0.0

        '''for k in [1, 2, 3]:
            if k == 1:
                print('RRT')
                self.ur5.set_planner_id("RRT")      # choice path planning algorithm for the arm
                self.gripper.set_planner_id("RRT")  # choice path planning algorithm for the gripper
            elif k == 2:
                print('RRTstar')
                self.ur5.set_planner_id("RRTstar")      # choice path planning algorithm for the arm
                self.gripper.set_planner_id("RRTstar")  # choice path planning algorithm for the gripper
            else:
                print('PRM')
                self.ur5.set_planner_id("PRM")      # choice path planning algorithm for the arm
                self.gripper.set_planner_id("PRM")  # choice path planning algorithm for the gripper
            
            for j in [1, 2, 3]:
                if j == 1:
                    print('pose1')
                    pose = pose1
                elif j == 2:
                    print('pose2')
                    pose = pose2
                else:
                    print('pose3')
                    pose = pose3
                i = 1; errors = []
                while(i < 31):
                    if self.step0():
                        self.ur5.set_pose_target(pose)  # set the target pose for the arm
                        start = rospy.get_time()
                        self.ur5.go(wait=True)  # plan and execute the position
                        self.ur5.stop()  # ensures that there is no residual movement
                        cur_pose = self.ur5.get_current_pose().pose
                        if self.all_close(pose, cur_pose):
                            current = rospy.get_time()
                            array1 = np.array(pose_to_list(pose)); array2 = np.array(pose_to_list(cur_pose))
                            error_p = 1000 * np.sqrt(((array1[:3] - array2[:3])**2).mean())       # [mm]
                            error_q = np.rad2deg(np.sqrt(((array1[3:] - array2[3:])**2).mean()))  # [rad]
                            errors.append([np.round(error_p, 4), np.round(error_q, 4), current - start])
                            #print(i)
                            i += 1
                errors = np.array(errors)
                means = [[np.mean(errors[:, 0]), np.mean(errors[:, 1]), np.mean(errors[:, 2])]]
                stds = [[np.std(errors[:, 0]), np.std(errors[:, 1]), np.std(errors[:, 2])]]
                errors = np.concatenate((errors, means, stds))
                errors = errors.tolist()
                print(tabulate(errors, headers=['position error [mm]', 'orientation error [deg]', 'time [sec]'],
                                tablefmt='fancy_grid', showindex=range(1, i) + ['avg', 'std'])) #, floatfmt=".4f"))'''
        
        # Plots - creating the dataset base on results from above code
        data_p1 = {'RRT':(0.0399, 0.0125), 'RRTstar':(0.0410, 0.0128), 'PRM':(0.0450, 0.0093)}
        data_p2 = {'RRT':(0.0435, 0.0135), 'RRTstar':(0.0465, 0.0095), 'PRM':(0.0455, 0.0134)}
        data_p3 = {'RRT':(0.0439, 0.0122), 'RRTstar':(0.0430, 0.0115), 'PRM':(0.0422, 0.0095)}

        data_o1 = {'RRT':(0.0166, 0.0028), 'RRTstar':(0.0154, 0.0031), 'PRM':(0.0158, 0.0034)}
        data_o2 = {'RRT':(0.0147, 0.0032), 'RRTstar':(0.0148, 0.0034), 'PRM':(0.0152, 0.0038)}
        data_o3 = {'RRT':(0.0144, 0.0041), 'RRTstar':(0.0156, 0.0030), 'PRM':(0.0151, 0.0033)}

        data_t1 = {'RRT':(8.6630, 4.2918), 'RRTstar':(11.8483, 2.3438), 'PRM':(12.2582, 2.1419)}
        data_t2 = {'RRT':(9.0660, 3.5811), 'RRTstar':(11.8388, 1.9913), 'PRM':(12.1324, 2.1299)}
        data_t3 = {'RRT':(7.4832, 2.5620), 'RRTstar':(11.8381, 2.4100), 'PRM':(11.8336, 2.5718)}

        avgs_p1 = [ele[0] for ele in list(data_p1.values())]; stds_p1 = [ele[1] for ele in list(data_p1.values())]
        avgs_p2 = [ele[0] for ele in list(data_p2.values())]; stds_p2 = [ele[1] for ele in list(data_p2.values())]
        avgs_p3 = [ele[0] for ele in list(data_p3.values())]; stds_p3 = [ele[1] for ele in list(data_p3.values())]
        
        avgs_o1 = [ele[0] for ele in list(data_o1.values())]; stds_o1 = [ele[1] for ele in list(data_o1.values())]
        avgs_o2 = [ele[0] for ele in list(data_o2.values())]; stds_o2 = [ele[1] for ele in list(data_o2.values())]
        avgs_o3 = [ele[0] for ele in list(data_o3.values())]; stds_o3 = [ele[1] for ele in list(data_o3.values())]

        avgs_t1 = [ele[0] for ele in list(data_t1.values())]; stds_t1 = [ele[1] for ele in list(data_t1.values())]
        avgs_t2 = [ele[0] for ele in list(data_t2.values())]; stds_t2 = [ele[1] for ele in list(data_t2.values())]
        avgs_t3 = [ele[0] for ele in list(data_t3.values())]; stds_t3 = [ele[1] for ele in list(data_t3.values())]

        algorithms = list(data_p1.keys())
        
        # Set position of bar on x axis
        barWidth = 0.04; bar = [0, 1, 2]
        bar1 = [x + barWidth/6.0 for x in bar]
        bar2 = [x + barWidth for x in bar1]
        bar3 = [x + barWidth for x in bar2]
        bar4 = [x + barWidth for x in bar3]
        bar5 = [x + barWidth for x in bar4]
        bar6 = [x + barWidth for x in bar5]

        # Make the plots
        plt.subplots(figsize =(8, 6))
        plt.bar(bar1, avgs_p1, width = barWidth, edgecolor ='grey', label ='pose1_average')
        plt.bar(bar2, stds_p1, width = barWidth, edgecolor ='grey', label ='pose1_standard deviation')
        plt.bar(bar3, avgs_p2, width = barWidth, edgecolor ='grey', label ='pose2_average')
        plt.bar(bar4, stds_p2, width = barWidth, edgecolor ='grey', label ='pose2_standard deviation')
        plt.bar(bar5, avgs_p3, width = barWidth, edgecolor ='grey', label ='pose3_average')
        plt.bar(bar6, stds_p3, width = barWidth, edgecolor ='grey', label ='pose3_standard deviation')
        # Adding Xticks
        plt.xlabel("algorithm", fontweight='bold', fontsize=10)
        plt.ylabel("position error [mm]", fontweight ='bold', fontsize=10)
        plt.xticks([r + barWidth for r in range(3)], algorithms)
        plt.legend(fontsize=9, loc='center right')
        plt.title("Position error depending on algorithm", fontweight='bold'); plt.show()

class move_group_interface():
    def __init__(self, object):
        moveit_commander.roscpp_initialize(sys.argv)    # init moveit_commander, allow to communicate with a move_group
        self.robot = moveit_commander.RobotCommander()  # provides information such as the robot kinematic model and the robot current joint states
        self.scene = moveit_commander.PlanningSceneInterface()  # provides a remote interface for getting, setting, and updating the robot internal understanding of the surrounding world
        self.ur5_group = moveit_commander.MoveGroupCommander("ur5")  # interface to a planning group (of joints), can be used to plan and execute motions
        self.gripper_group = moveit_commander.MoveGroupCommander("robotiq85")
        self.ur5_group.set_planner_id("RRT")      # choice RRT-path planning algorithm for the arm
        self.gripper_group.set_planner_id("RRT")  # choice RRT-path planning algorithm for the gripper
        self.eef_link = self.ur5_group.get_end_effector_link()  # name of the end-effector link for the ur5    
        self._object = object
    
    def wait_for_state_update(self, garbage_is_known=False, garbage_is_attached=False, timeout=8.0):  # Ensuring collision updates are receieved
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():  # test if the garbage is in attached objects
            attached_objects = self.scene.get_attached_objects([self._object._name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = self._object._name in self.scene.get_known_object_names()  # test if the garbage is in the scene. note that attaching the garbage will remove it from known_objects
            
            if (garbage_is_attached == is_attached) and (garbage_is_known == is_known):  # test if we are in the expected state
                return True
            rospy.sleep(0.2)   # sleep so that we give other threads time on the processor
            seconds = rospy.get_time()
        return False  # if exited the while loop without returning, then it timed out
    
    def add_garbage_to_Rviz(self):
        rospy.sleep(2)  # to ensure the scene interface is connected to the move_group. the connection is created in the background
        garb_pose = PoseStamped()
        garb_pose.header.frame_id = "map"  # map is the gloabal frame
        garb_pose.pose.position.x = self._object.garbage_point[0]
        garb_pose.pose.position.y = self._object.garbage_point[1]
        garb_pose.pose.position.z = 0.0    # in Rviz the frame is in the ground
        garb_pose.pose.orientation.x = self._object.orientation[0]
        garb_pose.pose.orientation.y = self._object.orientation[1]
        garb_pose.pose.orientation.z = self._object.orientation[2]
        garb_pose.pose.orientation.w = self._object.orientation[3]
        self.scene.add_cylinder(self._object._name, garb_pose, height=self._object.H, radius=self._object.R) # add the cylinder to Rviz-the planning scene
        # self.scene.add_box("box", garb_pose, size=(0.110998, 0.077469, 0.084021))
        print("the garbage was added to Rviz")
        return self.wait_for_state_update(garbage_is_known=True)

    def attach_garbage(self):
        touch_links = self.robot.get_link_names(group="robotiq85")
        self.scene.attach_mesh(self.eef_link, self._object._name, touch_links=touch_links)   # telling the planning scene to ignore collisions between the gripper and the garbage.
        print("the garbage was attached")
        return self.wait_for_state_update(garbage_is_known=False, garbage_is_attached=True)  # wait for the planning scene to update.

    def detach_garbage(self):
        self.scene.remove_attached_object(self.eef_link, name=self._object._name)
        return self.wait_for_state_update(garbage_is_known=True, garbage_is_attached=False)  # wait for the planning scene to update
    
    def remove_garbage_from_Rviz(self):  # removing objects from the Planning Scene
        self.scene.remove_world_object(self._object._name)  # Note: The object must be detached before remove it from the world
        return self.wait_for_state_update(garbage_is_known=False, garbage_is_attached=False)  # wait for the planning scene to update

    def plot_workspace(self, num_points):  # plot X_free
        '''P = []  # list for store 3D points that the ee can to be in them
        i = 1
        self.ur5_group.set_rpy_target([np.pi/2, 0.0, 0.0])
        self.ur5_group.go(wait=True)  # plan and execute the position
        self.ur5_group.stop()  # ensures that there is no residual movement
        print(i)
        cur_position = self.ur5_group.get_current_pose(self.eef_link).pose.position
        if cur_position.z > 0:  # up to floor
            P.append(np.array([cur_position.x, cur_position.y, cur_position.z]))
            i += 1
        while(i <= num_points):
            ### return the husky to start pose ###
            origin_pose = ModelState()
            origin_pose.model_name = 'husky'
            origin_pose.pose.position.x, origin_pose.pose.position.y, origin_pose.pose.position.z = 0.0, 0.0, 0.132266
            q = quaternion_from_euler(0.0, 0.0, 0.0, 'ryxz')
            origin_pose.pose.orientation = Quaternion(*q)
            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                set_state(origin_pose)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

            print(i)
            cur_pose = self.ur5_group.get_current_pose().pose
            if cur_pose.position.z < 0.15:   # avoid collision with the husky's bottom
                self.ur5_group.set_named_target("home")
            elif cur_pose.position.z < 0.8:  # for go out from privious pose
                cur_pose.position.z += 0.005
                self.ur5_group.set_pose_target(cur_pose)
            else:
                cur_pose.position.z -= 0.005
                self.ur5_group.set_pose_target(cur_pose)
            self.ur5_group.go(wait=True)  # plan and execute the position

            self.ur5_group.set_rpy_target([np.pi/2, 0.0, 0.0])
            self.ur5_group.go(wait=True)  # plan and execute the position
            self.ur5_group.stop()  # ensures that there is no residual movement
            cur_position = self.ur5_group.get_current_pose(self.eef_link).pose.position
            if cur_position.z > 0:  # up to floor
                P.append(np.array([cur_position.x, cur_position.y, cur_position.z]))
                i += 1
        P = np.array(P)'''
        data1 = pickle.load(open("maps/5000points_90_0_90.dat", "rb"))  # load the data that was analyzed by the code above
        data1 = np.array(data1)
        # P = np.concatenate((P, data1), axis=0)
        print("length P: " + str(len(data1)))
        print("max coordinates:"); print(np.vstack(data1).max(axis=0))
        print("min coordinates:"); print(np.vstack(data1).min(axis=0))
        plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot3D(data1[:, 0], data1[:, 1], data1[:, 2], '.b', markersize=0.5)
        ax.view_init(elev=90, azim=-90)
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_zlabel('z [m]')
        
        # circle
        circle = plt.Circle((-0.005, 0.0), 0.006, color='r', fill=False)
        ax.add_artist(circle)
        # ax.scatter3D(0.33, 0.0, 0.0, color='black')  # circle's center, for 90-0-0
        ax.scatter3D(0.24, 0.1, 0.0, color='black')  # circle's center, for 90-0-90
        ax.scatter3D(0.052, 0.0, 0.0, color='red')   # base_link frame center
        
        plt.title('Workspace area with RPY=(90, 0, 90)'); plt.show()

def main():
    garb = garbage('obj0')  # the garbages are named obj0, obj1, ... obj(n-1)
    ############ move_group ###############################
    move_group = move_group_interface(garb)
    pp_process = pick_place(move_group.ur5_group, move_group.gripper_group, garb)
    pp_process.step0()  # send the arm to home position, for avoid collision of the hand with the obstacle
    ############ move_base ################################
    husky_distance = 1.3  # =0.65*2, the primary distance from the garbage that the husky will be scheduled to arrive
    distance = np.linalg.norm(garb.garbage_point - pp_process.curr_xy_base_link)
    print(distance)
    if distance > husky_distance:
        result = pp_process.send_husky(husky_distance, pp_process.curr_xy_base_link, pp_process._object, pp_process.husky_target_xy)  # send husky close to the garbage
        if result:
            rospy.loginfo("goal execution done!")
            result2 = pp_process.send_husky(husky_distance/2.0, pp_process.curr_xy_base_link, pp_process._object, pp_process.husky_target_xy)  # send husky closer for picking
            if result2:
                rospy.loginfo("goal execution done!")
    else:
        result3 = pp_process.send_husky(husky_distance/2.0, pp_process.curr_xy_base_link, pp_process._object, pp_process.husky_target_xy)  # send husky closer for picking
        if result3:
            rospy.loginfo("goal execution done!")
    ############ pick and place process by steps ##########
    '''move_group.scene.remove_world_object('obj0')  # for re-run this script
    rospy.sleep(2)     
    pp_process.step1_1()  # open the gripper'''
    # move_group.plot_workspace(0)

    '''if pp_process.step1_2():          # bring the arm above the garbage
        if pp_process.step1_3():      # bring the arm down
            if pp_process.step1_4():  # bring the arm closer
                if move_group.add_garbage_to_Rviz():
                    if move_group.attach_garbage():
                        if pp_process.step1_5():      # close the gripper
                            while not pp_process.step2_1():
                                print(u"\u001b[31m---- step 2_1 failed! ----")
                            if pp_process.step2_2():  # place in the dedicated tin
                                print(u"\u001b[32m---- the pick and place proccess was success! ----")
                            else:
                                print(u"\u001b[31m---- step 2_2 failed! ----")
                            
                            if pp_process.step0():      # pick up the garbage
                                if pp_process.step2_2():  # place in the dedicated tin
                                    print(u"\u001b[32m---- the pick and place proccess was success! ----")
                                else:
                                    print(u"\u001b[31m---- step 2_2 failed! ----")
                            else:
                                print(u"\u001b[31m---- step 2_1 failed! ----")
                        else:
                            print(u"\u001b[31m---- step 1_5 failed! ----")
                    else:
                        print(u"\u001b[31m---- attach_garbage failed! ----")
                else:
                    print(u"\u001b[31m---- add_garbage failed! ----")
            else: 
                print(u"\u001b[31m---- step 1_4 failed! ----")
        else:
            print(u"\u001b[31m---- step 1_3 failed! ----")
    else:
        print(u"\u001b[31m---- step 1_2 failed! ----")'''

    def ompl_obstacles_avoidance():  # analyze the ompl planner for the arm and gripper with obstacles avoidance
        pose1 = Pose()
        pose1.position.x, pose1.position.y, pose1.position.z = 0.7, 0.0, 0.2
        # RPY = (90, 0, 0):
        pose1.orientation.x, pose1.orientation.y = sqrt(0.5), 0.0
        pose1.orientation.z, pose1.orientation.w = 0.0, sqrt(0.5)
        # add the obsticle to Rviz
        rospy.sleep(2)
        garb_pose0 = PoseStamped()
        garb_pose0.header.frame_id = "map"
        garb_pose0.pose.position.x = garb.garbage_point[0]; garb_pose0.pose.position.y = garb.garbage_point[1]; garb_pose0.pose.position.z = 0.0
        garb_pose0.pose.orientation.x = garb.orientation[0]; garb_pose0.pose.orientation.y = garb.orientation[1]
        garb_pose0.pose.orientation.z = garb.orientation[2]; garb_pose0.pose.orientation.w = garb.orientation[3]
        move_group.scene.add_cylinder(garb._name, garb_pose0, height=garb.H, radius=garb.R)

        '''obstacle_means = []; obstacle_stds = []
        for k in [1, 2, 3]:
            if k == 1:
                print('RRT')
                move_group.ur5_group.set_planner_id("RRT")      # choice path planning algorithm for the arm
                move_group.gripper_group.set_planner_id("RRT")  # choice path planning algorithm for the gripper
            elif k == 2:
                print('RRTstar')
                move_group.ur5_group.set_planner_id("RRTstar")      # choice path planning algorithm for the arm
                move_group.gripper_group.set_planner_id("RRTstar")  # choice path planning algorithm for the gripper
            else:
                print('PRM')
                move_group.ur5_group.set_planner_id("PRM")      # choice path planning algorithm for the arm
                move_group.gripper_group.set_planner_id("PRM")  # choice path planning algorithm for the gripper

            i = 1; errors = []
            while(i < 31):
                if pp_process.step0():
                    ### return obj0 to start pose ###
                    origin_pose = ModelState()
                    origin_pose.model_name = 'obj0'
                    origin_pose.pose.position.x, origin_pose.pose.position.y, origin_pose.pose.position.z = 0.65, 0.2, 0.165476
                    q = quaternion_from_euler(0.0, 0.0, 0.0, 'ryxz')
                    origin_pose.pose.orientation = Quaternion(*q)
                    rospy.wait_for_service('/gazebo/set_model_state')
                    try:
                        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                        set_state(origin_pose)
                    except rospy.ServiceException as e:
                        print("Service call failed: %s" % e)

                    ### return the husky to start pose ###
                    origin_pose2 = ModelState()
                    origin_pose2.model_name = 'husky'
                    origin_pose2.pose.position.x, origin_pose2.pose.position.y, origin_pose2.pose.position.z = 0.0, 0.0, 0.132266
                    q2 = quaternion_from_euler(0.0, 0.0, 0.0, 'ryxz')
                    origin_pose2.pose.orientation = Quaternion(*q2)
                    rospy.wait_for_service('/gazebo/set_model_state')
                    try:
                        set_state2 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                        set_state2(origin_pose2)
                    except rospy.ServiceException as e:
                        print("Service call failed: %s" % e)
                    rospy.sleep(1)

                    move_group.ur5_group.set_pose_target(pose1)  # set the target pose for the arm
                    start = rospy.get_time()
                    move_group.ur5_group.go(wait=True)  # plan and execute the position
                    move_group.ur5_group.stop()  # ensures that there is no residual movement
                    cur_pose = move_group.ur5_group.get_current_pose().pose
                    if pp_process.all_close(pose1, cur_pose):
                        current = rospy.get_time()
                        array1 = np.array(pose_to_list(pose1)); array2 = np.array(pose_to_list(cur_pose))
                        error_p = 1000 * np.sqrt(((array1[:3] - array2[:3])**2).mean())       # [mm]
                        error_q = np.rad2deg(np.sqrt(((array1[3:] - array2[3:])**2).mean()))  # [rad]
                        errors.append([np.round(error_p, 4), np.round(error_q, 4), current - start])
                        #print(i)
                        i += 1
            errors = np.array(errors)
            means = [[np.mean(errors[:, 0]), np.mean(errors[:, 1]), np.mean(errors[:, 2])]]
            stds = [[np.std(errors[:, 0]), np.std(errors[:, 1]), np.std(errors[:, 2])]]
            errors = np.concatenate((errors, means, stds))
            errors = errors.tolist()
            print(tabulate(errors, headers=['position error [mm]', 'orientation error [deg]', 'time [sec]'],
                            tablefmt='fancy_grid', showindex=range(1, i) + ['avg', 'std'])) #, floatfmt=".4f"))
            obstacle_means.append(means[0])
            obstacle_stds.append(stds[0])'''
        
        # Plots - creating the dataset
        '''obstacle_avgs_p = [x[0] for x in obstacle_means]; obstacle_stds_p = [y[0] for y in obstacle_stds]
        obstacle_avgs_o = [x[1] for x in obstacle_means]; obstacle_stds_o = [y[1] for y in obstacle_stds]
        obstacle_avgs_t = [x[2] for x in obstacle_means]; obstacle_stds_t = [y[2] for y in obstacle_stds]'''
      

        # after collection data by the above code:
        data_p = {'RRT':(0.0466733, 0.00876721), 'RRTstar':(0.04209000, 0.0106978), 'PRM':(0.0479733, 0.0108231)}
        data_o = {'RRT':(0.01635000, 0.00350331), 'RRTstar':(0.01479330, 0.00286763), 'PRM':(0.01574330, 0.00324188)}
        data_t = {'RRT':(8.25847, 2.92407), 'RRTstar':(11.71740, 1.91898), 'PRM':(11.28720, 2.25288)}

        avgs_p = [x[0] for x in list(data_p.values())]; stds_p = [y[1] for y in list(data_p.values())]
        avgs_o = [x[0] for x in list(data_o.values())]; stds_o = [y[1] for y in list(data_o.values())]
        avgs_t = [x[0]/60.0 for x in list(data_t.values())]; stds_t = [y[1]/60.0 for y in list(data_t.values())]

        algorithms = list(data_p.keys())

        # set position of bar on x axis
        barWidth = 0.04; bar = [0, 1, 2]
        bar1 = [x + barWidth/6.0 for x in bar]
        bar2 = [x + barWidth for x in bar1]
        bar3 = [x + barWidth for x in bar2]
        bar4 = [x + barWidth for x in bar3]
        bar5 = [x + barWidth for x in bar4]
        bar6 = [x + barWidth for x in bar5]

        # make the plots
        plt.subplots(figsize = (8, 6))
        plt.bar(bar1, avgs_p, width = barWidth, edgecolor ='grey', label ='position_error_avg [mm]')
        plt.bar(bar2, stds_p, width = barWidth, edgecolor ='grey', label ='position_error_std [mm]')
        plt.bar(bar3, avgs_o, width = barWidth, edgecolor ='grey', label ='orientation_error_avg [deg]')
        plt.bar(bar4, stds_o, width = barWidth, edgecolor ='grey', label ='orientation_error_std [deg]')
        plt.bar(bar5, avgs_t, width = barWidth, edgecolor ='grey', label ='time_avg [min]')
        plt.bar(bar6, stds_t, width = barWidth, edgecolor ='grey', label ='time_std [min]')
        # adding Xticks
        plt.xlabel("algorithm", fontweight='bold', fontsize=10)
        plt.ylabel("comparison parameter", fontweight ='bold', fontsize=10)
        plt.xticks([r + barWidth for r in range(3)], algorithms)
        plt.legend(fontsize=9, loc='center right')
        plt.title("Pose1: comparison depending on algorithm, with obstacles avoidance", fontweight='bold'); plt.show()
    
    #pp_process.ompl_planner_analysis()
    #ompl_obstacles_avoidance()
    ############ clear and reset ##########################
    #print(move_group.detach_garbage())
    move_group.ur5_group.clear_pose_targets(); move_group.gripper_group.clear_pose_targets()

if __name__ == '__main__':  # if the python node is executed as main process (sourced directly)
    rospy.init_node('pp_one_node', anonymous=True)  # init a node to let publish and subscribe
    main()
    rospy.sleep(2)
    rospy.signal_shutdown("ctrl+c was typed")
    moveit_commander.roscpp_shutdown()
