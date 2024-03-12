# import neccesry packges
import numpy as np 
from matplotlib import pyplot as plt 
from PIL import Image 
from math import sqrt
import random
# class that represent a node in the RRT tree
class Node:
    def __init__(self , x , y  ):
        self.x = x # x-position
        self.y = y # y-postion
        self.id = 0 # vertices id 
        self.f_score = float('inf') # initialize as inifinity 
        self.g_score = float('inf') # initialize as inifinity 
        self.parent  = None
    # calculate the huristic value of the node
    def calcu_huristic(self,target):
        distance = sqrt( (self.x-target.x)**2 + (self.y-target.y)**2 )
        return distance 
    # calculate the distance between the current node and the target node
    def get_distance(self,target):
        distance = sqrt( (self.x-target.x)**2 + (self.y-target.y)**2 )
        return distance 
    # find the nearest node from the given nodes with in the radius for RRT* method
    def find_nearest_node(self,nodes):
        min_distance = float('inf')
        nearest_node = None
        for node in nodes:
            distance = self.get_distance(node)
            if(distance < min_distance):
                nearest_node = node
                min_distance = distance
            
        return nearest_node
    # filter the nodes with in the given radius used for RRT* method
    def find_nodes_with_in_radius(self,nodes,radius):
        nodes_with_in_radius =[]
        for node in nodes:      
            distance = self.get_distance(node)
            if(distance <= radius):
              nodes_with_in_radius.append(node)
        return nodes_with_in_radius
    def __str__(self):
        return str(self.id)
# class that represent the RRT tree
class RRT:
    def __init__(self,svc,k,q,p,dominion = [-10,10,-10,10] ,max_time = 0.1, is_RRT_star = True):
 
        self.svc      = svc
        self.k        = k
        self.q        = q
        self.p        = p 
        self.dominion = dominion
        self.max      = max_time 
        self.vertices =      [ ]
        self.edges    =      [ ]
        self.vertices_star = [ ] 
        self.edges_star    = [ ]
        self.node_counter  =  1
        self.path          = [ ]
        self.smoothed_path = [ ]
        self.is_RRT_star = is_RRT_star # by deafault it is False we implement RRT
        self.radius = 2 # radius for RRT* search  method

    # Finds th  optimal node parent  with in given radius 
    # used for RRT* Cost Functionality 
    def cost_optimal_parent(self,qnew,current_parent):
        # filter a nodes with a given radius 
        nodes_with_in_radius = qnew.find_nodes_with_in_radius(self.vertices,self.radius)
        best_cost = current_parent.g_score + qnew.get_distance(current_parent)
        best_parent = current_parent
        if(not nodes_with_in_radius): # return the nearest node
            return best_parent
        else :
            for node in nodes_with_in_radius:
                new_node_cost = node.g_score + qnew.get_distance(node)
                if(new_node_cost < best_cost and self.svc.check_path([[node.x,node.y],[qnew.x , qnew.y]])):
                    best_parent = node

            return best_parent

    def rewire(self,qnew):
        # filter a nodes with a given radius 
        nodes_with_in_radius = qnew.find_nodes_with_in_radius(self.vertices,self.radius)
        for node in  nodes_with_in_radius:
            new_node_cost = qnew.g_score + qnew.get_distance(node)      
        
            if(new_node_cost < node.g_score and self.svc.check_path([[node.x,node.y],[qnew.x , qnew.y]])):
                node.parent = qnew
                node.g_score = new_node_cost
                node.f_score = node.g_score + node.get_distance(self.goal)

    def Rand_Config(self):       
        
        prob = random.random() # generate a random number between 0 and 1
        # generate a random node with in the dominion 
        x = random.uniform(self.dominion[0],self.dominion[1])
        y = random.uniform(self.dominion[2],self.dominion[3])
        qrand = Node(x,y)
        # if the random number is less than the probability of selecting the goal
        if(prob < self.p):          
           qrand = self.goal # set initialy qrand as goal

        return qrand            
    def Near_Vertices(self , qrand):
        qnear =  qrand.find_nearest_node(self.vertices ) # find the nearest node from the vertices
        return qnear
    def New_Config(self , qnear , qrand):
        dir_vector = np.array([qrand.x -  qnear.x , qrand.y - qnear.y])
        length = qnear.get_distance(qrand)
        norm_vector = dir_vector/length
        if(self.q > length):
            return qrand
        qnew = np.array([qnear.x,qnear.y]) + norm_vector*self.q      
        qnew = Node(qnew[0] , qnew[1])
        return qnew
    
    def reconstract_path(self):
        current = self.goal
        self.path = [current]
        while( current != self.start):
            current = current.parent
            self.path.append(current)
        path =[(n.x,n.y) for n in self.path]
        self.path.reverse()
        path =[[n.x,n.y] for n in self.path]
        
        return path
        
    def smooth_path(self):
        counter = 0
        max_iterations = 100
        self.smoothed_path = [self.goal]
        while True:
            for node in self.path:
                next_path = self.smoothed_path[len(self.smoothed_path)-1]
                if(self.svc.check_path([[node.x , node.y],[next_path.x, next_path.y]])):
    
                    self.smoothed_path.append(node)
                    break
            if self.smoothed_path[len(self.smoothed_path)-1] == self.start:
                break
            counter +=1
        if(counter >= max_iterations):
            print("max iteration reached")
            self.smoothed_path = self.path

        self.smoothed_path.reverse()
        path =[(n.x,n.y) for n in self.smoothed_path]
        
        return path
    
    def get_tree(self):
        tree_list = [[[edge[0].x , edge[0].y] ,[edge[1].x , edge[1].y]] for edge in self.edges]
        return tree_list
    def compute_path(self , start , goal):

        self.start = Node(start[0],start[1])   
        self.goal =  Node(goal[0],goal[1])
        self.vertices.append(self.start) # initialize the vertices list 
        self.start.g_score = 0
        self.start.f_score = self.start.g_score + self.start.calcu_huristic(self.goal)
        for k in range(self.k):
            qrand = self.Rand_Config()
            while(not self.svc.is_valid([qrand.x,qrand.y])):
               qrand = self.Rand_Config()
          
            qnear = self.Near_Vertices(qrand)
            qnew  = self.New_Config(qnear,qrand)
      
            if( self.svc.check_path([[qnear.x ,qnear.y ] ,[qnew.x , qnew.y] ])):    
                
                if(self.is_RRT_star == True): # if is RRT star is false we implemet Cost function
                     # We Select new parent with optimal cost
                     qnear = self.cost_optimal_parent(qnew,qnear)

                self.vertices.append(qnew)
                self.edges.append((qnear,qnew))
                qnew.parent = qnear 
                qnew.id = self.node_counter 
                qnew.g_score = qnear.g_score + qnew.get_distance(qnear)
                qnew.f_score = qnear.g_score + qnew.calcu_huristic(self.goal)

                # Implement rewire here
                if(self.is_RRT_star == True): # if is RRT star is true we implemet rewire function
                    self.rewire(qnew)

                if(qnew == self.goal):
                    self.reconstract_path()
                    # self.smoothed_path
                    return  self.smooth_path() , self.get_tree() 
                    
    
                self.node_counter +=1
        return [], self.get_tree()
