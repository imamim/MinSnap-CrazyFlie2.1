#RAHMAN VE RAHIM OLAN ALLAH'IN ADIYLA, MUVAFFAKIYETMIZ YALNIZCA O'NA AITTIR!
import rospy
from State import Position,Velocity,DesiredTrajectoryState
from UavClass import UavClass
import pybullet as p
from threading import Lock
from minsnap.msg import FullState, FullCommand, FullTrajectoryCommand
from geometry_msgs.msg import Pose


class MinsnapManager:
    def __init__(self,init_params,initial_position):
        self.init_params = init_params
        self.initial_position_list = initial_position["PoseArray"]
        self.initial_orientation_list = initial_position["OrientationArray"]
        self.mutex = Lock()
        self.uav = None

        self.initialize()

        
    def initialize(self):
        rospy.init_node('minsnap')
        self.pub = rospy.Publisher("uav_commands",FullCommand,queue_size=10)
        self.trajectory_pub = rospy.Publisher("uav_trajectory_commands", FullTrajectoryCommand, queue_size=10)
        
        uav_class = UavClass("uav/"+str(1),self.initial_position_list[0],self.initial_orientation_list[0])
        self.uav = uav_class


        #If and only simulation enable, subscribe simulation data.
        if self.init_params.simulation_enabled and not self.init_params.real_enabled:
            rospy.Subscriber("simulation_uav_state",FullState,self.simStateCallback)

        #If and only real enabe, subscribe real data.
        elif not self.init_params.simulation_enabled and self.init_params.real_enabled:
            rospy.Subscriber("real_uav_state", FullState, self.realStateCallback)



    def simStateCallback(self,data):
        self.mutex.acquire()
        for msg in data.fullState:
            index = int(msg.id[4:])-1 # Equal = 1
            euler = p.getEulerFromQuaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            position = Position(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,euler[0],euler[1],euler[2])
            velocity = Velocity(msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z,msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z)
            self.uav.activation_flag = msg.active
            self.uav.current_velocity = velocity
            self.uav.current_position = position
        self.mutex.release()
        


    def realStateCallback(self,data):
        self.mutex.acquire()
        for msg in data.fullState:
            index = int(msg.id[4:])-1
            euler = p.getEulerFromQuaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            position = Position(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,euler[0],euler[1],euler[2])
            velocity = Velocity(msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z,msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z)
            self.uav.activation_flag = msg.active
            self.uav.current_velocity = velocity
            self.uav.current_position = position
        self.mutex.release()



    def publishCommand(self):
        msg = FullCommand()
        msg.allcommands.append(self.uav.command_message)
        self.pub.publish(msg)

        
    def publishTrajectoryCommand(self):
        msg = FullTrajectoryCommand()
        
        msg.allcommands.append(self.uav.trajectory_command_message)
        self.trajectory_pub.publish(msg)



    
