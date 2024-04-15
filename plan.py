#!/usr/bin/env python3

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates,LinkStates
from geometry_msgs.msg import Pose2D,Twist
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates,LinkStates
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse



def rot_2d(theta):

    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    return np.array([[cos_theta, -sin_theta],
                     [sin_theta, cos_theta]])


def talker():
    pub_state = rospy.Publisher('/robot_control/start',Bool,queue_size=1)
    pub_pose = rospy.Publisher('/robot_control/goal_point', Pose2D, queue_size=1)

    rospy.init_node('cmd_robot', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        state = True
        # state.data = True

        pose = Pose2D()
        pose.x = 1.0
        pose.y = 1.0
        pose.theta = 0.0

        # rospy.loginfo(pose)
        pub_state.publish(state)
        pub_pose.publish(pose)
        rate.sleep()
 
class RobotControl:
    def __init__(self):

        self.goal = [0,0,0]
        self.start_pose = [0,0,0]
        self.current_pose = [0,0,0]
        self.robot_name = "robot"
        self.start_tag = False


        rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)
        rospy.Subscriber('/robot_control/goal_point', Pose2D, self.goal_pose_callback)
        rospy.Subscriber('/robot_control/start', Bool, self.start_callback)
        rospy.Subscriber('/robot_control/gripper', Bool, self.gripper_callback)

        

    def start_callback(self,data):
        if data.data:
            self.start_tag = True
        else:
            if self.start_tag == True:
                self.reset_world()
                self.start_tag = False

    def gripper_callback(self,data):
        self.gazebo_grip_cal(data.data)

    def gazebo_grip_cal(self,griping):
        
        if self.gripper_model_link == None and griping:
            gazebo_ls = rospy.wait_for_message("/gazebo/link_states", LinkStates, timeout=None)
            obj_name = 'ball'
            gripper_link = 'gripper'
            
            gripper_pos = []
            object_pos = []
            # print(f"gazebo_ls.name : {gazebo_ls.name}")
            for order,name in enumerate(gazebo_ls.name):
                
                if gripper_link in name:
                    gripper_pos = [name,[gazebo_ls.pose[order].position.x,
                                            gazebo_ls.pose[order].position.y,
                                            gazebo_ls.pose[order].position.z]]
                    
                if obj_name in name:
                    object_pos.append([name,[gazebo_ls.pose[order].position.x,
                                            gazebo_ls.pose[order].position.y,
                                            gazebo_ls.pose[order].position.z]])
            
            # print(f"gripper_pos : {gripper_pos}")
            # print(f"object_pos : {object_pos}")
            
            nearest_obj_buffer = None
            nearest_id = None
            for i,(name,pos) in enumerate(object_pos):
                vector = np.subtract(gripper_pos[1],pos)
                distance = abs(np.linalg.norm(vector))
                if nearest_obj_buffer == None:
                    nearest_obj_buffer = distance
                    nearest_id = i
                elif nearest_obj_buffer>distance:
                    nearest_obj_buffer = distance
                    nearest_id = i
            # print(f"nearest_obj : {object_pos[nearest_id][0], nearest_obj_buffer}")
            if nearest_obj_buffer < 0.095:
                req = AttachRequest()
                self.gripper_model_link = gripper_pos[0].split('::')
                req.model_name_1 = self.gripper_model_link[0]
                req.link_name_1 = self.gripper_model_link[1]

                self.object_model_link = object_pos[nearest_id][0].split('::')
                req.model_name_2 = self.object_model_link[0]
                req.link_name_2 = self.object_model_link[1]

                self.attach_srv.call(req)

        elif self.gripper_model_link != None and not griping :
            req = AttachRequest()
            
            req.model_name_1 = self.gripper_model_link[0]
            req.link_name_1 = self.gripper_model_link[1]

            req.model_name_2 = self.object_model_link[0]
            req.link_name_2 = self.object_model_link[1]

            self.detach_srv.call(req)

            self.object_model_link = None
            self.gripper_model_link = None

    def goal_pose_callback(self, data):
        if self.start_tag:
            self.goal = [data.x,data.y,data.theta]
            pose_goal = np.array(self.goal)

            pos_goal = pose_goal[:2]
            yaw_goal = pose_goal[2]

            # Yaw
            yaw_error = yaw_goal - self.current_pose[2]
            if abs(yaw_error)>np.pi:
                if yaw_error>0:
                    yaw_error = yaw_error-(2*np.pi)
                else:
                    yaw_error = yaw_error+(2*np.pi)
            yaw_output = self.yaw_pid.compute(yaw_error)
            
            print(f"===")
            print(f"yaw_goal : {yaw_goal:0.2f}")
            print(f"current_pose : {self.current_pose[2]:0.2f}")
            print(f"yaw_output : {yaw_output:0.2f}")


            # pos

            pose_goal_robot = np.subtract(pos_goal,self.current_pose[:2])
            pos_error = np.linalg.norm(pose_goal_robot)

            control_output = self.pos_pid.compute(pos_error)
            unit_vec = pose_goal_robot/pos_error
            out_vec = unit_vec*control_output

            out_pos = np.matmul(rot_2d(-self.current_pose[2]),out_vec.T)

            print(f"===")
            print(f"pos_goal : {pos_goal}")
            print(f"self.current_pose[:2] : {self.current_pose[:2]}")
            print(f"pose_goal_robot : {pose_goal_robot}")
            print(f"pos_error : {pos_error}")
            print(f"out_vec : {out_vec}")

            # self.move_robot(0,0,yaw_output)
            self.move_robot(out_pos[0],out_pos[1],yaw_output)

    def pose_callback(self, data):
        try:
            # print(data.name)
            idx = data.name.index(self.robot_name)
            basket_pose = np.zeros((5,2))
            red_ball_pose = np.zeros((6,2))
            purple_ball_pose = np.zeros((10,2))
            robot_pose = np.zeros((1,2))

            for i in range(6,11):
                pose = data.pose[i]
                for j in range(2):
                    if(j== 0):
                        basket_pose[i-6][j] = pose.position.x
                        # print(basket_pose[i-6][j])
                    elif(j==1):
                        basket_pose[i-6][j] = pose.position.y
                        # print(basket_pose[i-6][j])         
            
            for i in range(12,18):
                pose = data.pose[i]
                for j in range(2):
                    if(j== 0):
                        red_ball_pose[i-12][j] = pose.position.x
                        # print(red_ball_pose[i-6][j])
                    elif(j==1):
                        red_ball_pose[i-12][j] = pose.position.y
                        # print(red_ball_pose[i-6][j]) 
                # print(i)
                # print("\t")

            for i in range(18,28):
                pose = data.pose[i]
                for j in range(2):
                    if(j== 0):
                        purple_ball_pose[i-18][j] = pose.position.x
                        # print(purple_ball_pose[i-6][j])
                    elif(j==1):
                        purple_ball_pose[i-18][j] = pose.position.y
                        # print(purple_ball_pose[i-6][j])
                # print(i)
                # print("\t")  

            pose = data.pose[idx]
            robot_pose[idx-28][0] = pose.position.x
            robot_pose[idx-28][1] = pose.position.y
            
            # print(idx)
            # print(f"pose : {pose}")
            delta_distance = np.subtract(red_ball_pose, robot_pose)
            print(f"ball_pose: \n{red_ball_pose} \n , robot_pose:\n {robot_pose} \n, delta_distance: \n{delta_distance} ")
            print("\n")

            

            quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
                # print ("end this loop call back")
            if not self.start_tag:
                self.start_pose = np.array([pose.position.x,pose.position.y, euler_from_quaternion(quat)[2]])
            else:
                current_world = np.array([pose.position.x,pose.position.y, euler_from_quaternion(quat)[2]])
                self.current_pose = np.subtract(current_world,self.start_pose)

        except ValueError:
            print("Robot '{}' not found in the model states.".format(self.robot_name))

    def move_robot(self, linear_x,linear_y, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('robot_position_control', anonymous=True)

        robot_control = RobotControl()
        # talker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
