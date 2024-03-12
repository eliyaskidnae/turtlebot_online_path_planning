#!/usr/bin/python3

import numpy as np
import rospy
import tf
import math
import random
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from utils_lib.online_planning import StateValidityChecker, move_to_point, compute_path , wrap_angle
class OnlinePlanner:
    # OnlinePlanner Constructor
    def __init__(self, gridmap_topic, odom_topic, cmd_vel_topic, bounds, distance_threshold):
        # ATTRIBUTES
        # List of points which define the plan. None if there is no plan
        self.path = []
        self.edges = []
        self.smooth_path = []
        # State Validity Checker object                                                 
        self.svc = StateValidityChecker(distance_threshold)
        # Current robot SE2 pose [x, y, yaw], None if unknown            
        self.current_pose = None
        # Goal where the robot has to move, None if it is not set                                                                   
        self.goal = None
        # Last time a map was received (to avoid map update too often)                                                
        self.last_map_time = rospy.Time.now()
        # Dominion [min_x_y, max_x_y] in which the path planner will sample configurations                           
        self.bounds = bounds
        # Tolerance for the distance between the robot and the goal                                        
        self.tolorance = 0.05
        # CONTROLLER PARAMETERS
        # Proportional linear velocity controller gain
        self.Kv = 0.5
        # Proportional angular velocity controller gain                   
        self.Kw = 2
        # Maximum linear velocity control action                   
        self.v_max = 0.2
        # Maximum angular velocity control action               
        self.w_max = 0.3                
        self.retry = 0 # Retry counter for planning failures
        self.planning = True
        self.path_not_found = True
        # PUBLISHERS
        # Publisher for sending velocity commands to the robot
        # self.cmd_pub = None # TODO: publisher to cmd_vel_topic
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # Publisher for visualizing the path to with rviz
        self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=1)
        self.tree_pub = rospy.Publisher('~tree_marker', Marker, queue_size=1)
        self.not_smooth_path_pub = rospy.Publisher('~not_smooth_path', Marker, queue_size=1)
        # SUBSCRIBERS
        self.gridmap = rospy.Subscriber(gridmap_topic, OccupancyGrid, self.get_gridmap)
        # self.odom_sub = None # TODO: subscriber to odom_topic  
        self.odom = rospy.Subscriber(odom_topic, Odometry, self.get_odom)
        # self.move_goal_sub = None # TODO: subscriber to /move_base_simple/goal published by rviz 
        self.move_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal, queue_size=1)
        self.cmd = Twist()
        # TIMERS
        # Timer for velocity controller
        rospy.Timer(rospy.Duration(0.1), self.controller)
    
    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])

        # TODO: Store current position (x, y, yaw) as a np.array in self.current_pose var.
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    # Goal callback: Get new goal from /move_base_simple/goal topic published by rviz 
    # and computes a plan to it using self.plan() method  
    def get_goal(self, goal):
        
        if self.svc.there_is_map:
            
            self.path_not_found = True
            self.retry = 0
            # TODO: Store goal (x,y) as a numpy aray in self.goal var and print it 
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])  
            print("goal point ",self.goal)
            if(not self.svc.is_valid(self.goal)):
                rospy.logwarn("Goal Point is not valid , please try again")
            elif(not self.svc.is_valid(self.current_pose[0:2])):      
                rospy.logwarn("Start Point is not valid , please try again")
                self.recovery_behavior() # move around to find a valid point
            else :
                print("Valid Goal Point !")
                self.planning = True
            # Plan a new path to self.goal
                self.plan()

        
    # Map callback: Gets the latest occupancy map published by Octomap server and update 
    # the state validity checker
 

    def recovery_behavior(self):

        pose = self.svc.not_valid_pose(self.current_pose[0:2])
        pose = self.svc.__map_to_position__(pose)
        psi_d = math.atan2(pose[1] - self.current_pose[1] , pose[0]- self.current_pose[0])

        print("recovery behavior", psi_d , self.current_pose[2])
        print(pose ,self.current_pose)
        print(math.degrees(psi_d) , math.degrees(self.current_pose[2]))
        angle = math.degrees(wrap_angle(psi_d - self.current_pose[2]))
        count = 0
        max_iterations = 100000
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if( -90 < angle < 90):
              print("move back ward to recover")
              v = -1
        else:
              print("move forward to recover")
              v = 1 
        start_time = rospy.Time.now()
        duration = 0.5
        while(rospy.Time.now() - start_time).to_sec() < duration:
            # print("publish")
            self.__send_commnd__(v,0)  
        print("recovery behavior done")
        self.__send_commnd__(0, 0)

    def get_gridmap(self, gridmap):
      
        # To avoid map update too often (change value '1' if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 1:            
            self.last_map_time = gridmap.header.stamp
            
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.svc.set(env, gridmap.info.resolution, origin)
            
            # check if the goal and robot current pose is valid
            # if(self.goal is not None and not self.svc.is_valid(self.goal)):
            #     rospy.logwarn("Goal Point is not valid , please try again")
            if(self.goal is not None and not self.svc.is_valid(self.current_pose[0:2])):
             rospy.logwarn("Start Point is not valid , please move around ")
             self.recovery_behavior() # move around to find a valid point
            

            # If the robot is following a path, check if it is still valid
            if self.path is not None and len(self.path) > 0:
                # create total_path adding the current position to the rest of waypoints in the path
                total_path = [self.current_pose[0:2]] + self.path
     
                # if path is not valid and is uknown_valid is false, replan a new path to the goal
                if(not self.svc.check_path(total_path) and self.svc.is_unknown_valid ):
                   
                   rospy.loginfo("Replan agian current  path is in valid ")

                #    self.__send_commnd__(0, 0)
                   
                   self.plan()
                # TODO: check total_path validity. If total_path is not valid replan a new path to the goal
            
            elif  self.goal is not None and self.svc.is_valid(self.goal) and len(self.path) == 0:
             
                if(self.retry < 3):
                    self.retry += 1
                    rospy.loginfo("Retry planning ")
                    self.plan()
                

    # Solve plan from current position to self.goal. 
    def plan(self):
        # Invalidate previous plan if available
        self.path = []
        if(not self.svc.is_valid(self.goal)):
            rospy.logwarn("Goal is not valid, Target place is not safe")
            self.planning = False

        # print("Compute new path") 
        # TODO: plan a path from self.current_pose to self.goal
        else : 
            self.path , self.edges = compute_path(self.current_pose , self.goal , self.svc , self.bounds)
            self.draw_tree()

            # TODO: If planning fails, consider increasing the planning time, retry the planning a few times, etc.
            ...

            if len(self.path) == 0  :
                self.path_not_found = True
                              
            else:
                rospy.loginfo("Path Found !")
                self.path_not_found = True
                self.retry = 0 # reset retry counter
                # Publish plan marker to visualize in rviz
               
                self.publish_path()
                # self.notsmo_path_publish() 
                # remove initial waypoint in the path (current pose is already reached)
                if self.path[0]:
                    del self.path[0]                   
            
    # This method is called every 0.1s. It computes the velocity comands in order to reach the 
    # next waypoint in the path. It also sends zero velocity commands if there is no active path.
    def controller(self, event):
        v = 0
        w = 0
       
        if len(self.path) > 0:
            distance_to_goal = self.distance_to_target(self.path[0])

            if (distance_to_goal< self.tolorance):
            # TODO: If current waypoint is reached with some tolerance move to the next waypoint. 
                # print("one way point reached " , self.path[0] , distance_to_goal)
                del self.path[0]
                
                if(len(self.path) == 0 ):
                    rospy.loginfo("Goal Point Reached !")
                    self.planning= False
                    self.path_not_found = False
                    self.goal = None
                    self.retry = 0
                
            else: # TODO: Compute velocities using controller function in utils_lib
              
                v , w = move_to_point(self.current_pose, self.path[0], self.Kv , self.Kw )
        # Publish velocity commands 
        self.__send_commnd__(v, w)
    

    # PUBLISHER HELPERS
    # Transform linear and angular velocity (v, w) into a Twist message and publish it
    def __send_commnd__(self, v, w):
       
        self.cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(self.cmd)

    # Publish a path as a series of line markers
        
    def draw_tree(self):
        if(len(self.edges) > 0):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = Marker.LINE_LIST
            m.ns = 'tree'
            m.action = Marker.DELETEALL
            m.lifetime = rospy.Duration(0)
            self.marker_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.04
            m.scale.y = 0.0
            m.scale.z = 0.0

            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            color_black = ColorRGBA()
            color_black.r = 0
            color_black.g = 0
            color_black.b = 1
            color_black.a = 1
            for edge in self.edges:
                for node in edge:
                    p = Point()
                    p.x = node[0]
                    p.y = node[1]
                    p.z = 0.0
                    m.points.append(p)
                    m.colors.append(color_black)
            self.tree_pub.publish(m)

    def publish_path(self):
        if len(self.path) > 1:
            print("Publish path!")
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            self.marker_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.06
            m.scale.y = 0.0
            m.scale.z = 0.0
            
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            
            color_red = ColorRGBA()
            color_red.r = 1
            color_red.g = 0
            color_red.b = 0
            color_red.a = 1
            color_blue = ColorRGBA()
            color_blue.r = 0
            color_blue.g = 0
            color_blue.b = 1
            color_blue.a = 1

            p = Point()
            p.x = self.current_pose[0]
            p.y = self.current_pose[1]
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)
            
            for n in self.path:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.marker_pub.publish(m)
    def distance_to_target(self , target):
        # print("distance_to_target:", self.current_pose[0] , target[0] , self.current_pose[1] , target[1])
        return  math.sqrt((self.current_pose[0] - target[0])**2 + (self.current_pose[1] - target[1])**2)

# MAIN FUNCTION
if __name__ == '__main__':
    rospy.init_node('turtlebot_online_path_planning_node')   
    node = OnlinePlanner('/projected_map', '/odom', '/cmd_vel', np.array([-10.0, 10.0, -10.0, 10.0]), 0.2)
    
    # Run forever
    rospy.spin()
