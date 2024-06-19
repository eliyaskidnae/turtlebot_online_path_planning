import numpy as np
import math
import random
from  utils_lib.rrt_modle import RRT


def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.2, is_unknown_valid=True , is_rrt_star = False):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        # set method has been called                          
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid  
        self.is_rrt_star = is_rrt_star  
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
        self.height = data.shape[0]
        self.width = data.shape[1]

    # computes distance between two positions 
    def get_distance( self , first_pose , second_pose):
        return math.sqrt((second_pose[0] - first_pose[0] )**2 + (second_pose[1] - first_pose[1]) **2)
    
    # Given a pose, returs true if the pose is not in collision and false othewise.

    def is_valid(self, pose): 
        # TODO: convert world robot position to map coordinates using method __position_to_map__
        # TODO: check occupancy of the vicinity of a robot position (indicated by self.distance atribute). 
        # Return True if free, False if occupied and self.is_unknown_valid if unknown. 
        # If checked position is outside the map bounds consider it as unknown.
        m = self.__position_to_map__(pose)
        grid_distance = int(self.distance/self.resolution)
        # add 6 points upper and to lower limit 
        lower_x , lower_y = m[0] - grid_distance , m[1] - grid_distance  
        for lx in range(0,2*grid_distance):
            for ly in range(0,2*grid_distance):
                pose = lower_x + lx, lower_y + ly              
                # if one of the position is not free return False  , stop the loop 
                if(self.is_onmap(pose)):                           
                    if(not self.is_free(pose)): 
                        return False     
                # if  position is not in the map bounds return  is_unknown_valid
                else:
                    if(not self.is_unknown_valid):
                        return False
        return True
    def min_dis_obstacle(self, p, distance=1):
        
        m = self.__position_to_map__(p)
        grid_distance = int(distance/self.resolution)
        # print(m, grid_distance)
        # add 6 points upper and to lower limit 
        lower_x , lower_y = m[0] - grid_distance, m[1] - grid_distance
        min_dis = float('inf')
        for lx in range(2*grid_distance):
            for ly in range(2*grid_distance):
                pose = lower_x + lx, lower_y + ly
                
                if(not self.is_onmap(pose) or not self.is_free(pose)):
                    distance =  np.linalg.norm(np.array(pose) - np.array(m)) 
                    
                    min_dis = min(min_dis, distance)
        
        return   min_dis

    

    def not_valid_pose(self, pose): 
        # returns pose that are not valid
        m = self.__position_to_map__(pose)
        grid_distance = int(self.distance/self.resolution)
        # add 6 points upper and to lower limit 
        lower_x , lower_y = m[0] - grid_distance , m[1] - grid_distance  
        for lx in range(0,2*grid_distance):
            for ly in range(0,2*grid_distance):
                pose = lower_x + lx, lower_y + ly              
                # if one of the position is not free return False  , stop the loop 
                if(self.is_onmap(pose)):                           
                    if(not self.is_free(pose)): 
                        return pose     
                # if  position is not in the map bounds return  is_unknown_valid
                else:
                    if(not self.is_unknown_valid):
                        return pose
        return None

    # Given a path, returs true if the path is not in collision and false othewise.
    def check_path(self, path):
        step_size = 0.5*self.distance
        valid = True
        # TODO: Discretize the positions between 2 waypoints with an step_size = 2*self.distance
        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True. 
        for index in range(len(path)-1):

            first_pose  = path[index] # initial path 
            second_pose = path[index+1] # next path 
            # direction vector from first_pose to second_pose
            dir_vector = np.array([second_pose[0] - first_pose[0] , second_pose[1] - first_pose[1]])
            distance = self.get_distance(first_pose , second_pose) # distance from first_pose to second_pose
            
            if distance == 0:
                norm_vect = np.array([0, 0])
            else:
                norm_vect = dir_vector / distance # Compute normal vector b/n two points
            discrtized_seg = np.array([first_pose]) # array of discritized segment 
            current = first_pose

            # while the distance b/n two poses is smaller than min distance 
            while(self.get_distance(second_pose , current) > step_size):          
                current  = current + norm_vect*step_size
                valid = self.is_valid(current)      
                # if one of the discritized poses are not vlid return false    
                if(not valid):
                    return False      
            # Finnally Check the goal point
            valid = self.is_valid(second_pose)

        # If each path is valid return true
        return valid
    # Transform position with respect the map origin to cell coordinates
    def __position_to_map__(self, p):      
        x , y  =  p # get x and y positions 
        m_x    =  (x - self.origin[0])/self.resolution  # x cell cordinate 
        m_y    =  (y - self.origin[1])/self.resolution  # y cell cordinate 
        return [round(m_x), round(m_y)]   
    def __map_to_position__(self, m):
        x ,y = m  
        p_x  = self.origin[0] + x * self.resolution  
        p_y  = self.origin[1] + y * self.resolution 
        return [p_x, p_y]
    
    def is_onmap(self,pose):    
        # checks if a given pose in grid is on the map 
        # if is is on the map return Truem else return is False 
        if( 0<=pose[0]< self.height and 0<= pose[1] < self.width):
            return True        
        else:
            return False
        # checks a given pose in grid is free  
    def is_free(self, pose): 
        # if is is free return True , which means  
        if self.map[pose[0],pose[1]] == 0 :
            return True, 
        #if it is unkown return opposite of is unkown valid 
        elif self.map[pose[0],pose [1]] == -1 : # return opposite of is unkown valid 
            return  self.is_unknown_valid
        # return false , if is is obstacle 
        return False   


# Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the 
# StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# The planner returns a path that is a list of poses ([x, y]).
def compute_path(start_p, goal_p, svc, bounds , max_time=1.0):
    # call rrt class to compute the path , which is imported from othe file
    rrt = RRT(svc ,10000 ,3, 0.2 , bounds, max_time , svc.is_rrt_star)
    # returns the smooth path and the tree list
    path  , tree_list = rrt.compute_path(start_p, goal_p )
    return path  , tree_list

# Controller: Given the current position and the goal position, this function computes the desired 
# lineal velocity and angular velocity to be applied in order to reah the goal.
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    
    # TODO: Use a proportional controller which sets a velocity command to move from current position to goal (u = Ke)
    # To avoid strange curves, first correct the orientation and then the distance. 
    # Hint: use wrap_angle function to maintain yaw in [-pi, pi]
    # This function should return only  linear velocity (v) and angular velocity (w)
    distance =  math.sqrt((goal[0] - current[0])**2 + (goal[1] - current[1])**2)
    psi_d = math.atan2(goal[1] - current[1] , goal[0]- current[0])
    w_d = Kw*wrap_angle(psi_d - current[2])
    v_d = Kv*distance
    # if the angle is greater than 0.05 , set the linear velocity to 0
    # this makes the robot to turn on the spot
    if( abs(psi_d - current[2]) > 0.05):
        v_d = 0
        w_d = Kw*wrap_angle(psi_d - current[2])
    return v_d, w_d

