# import neccesry packges
import numpy as np 
from matplotlib import pyplot as plt 
from PIL import Image 
from math import sqrt
from random import random , randint  

# class 
class Node:
    def __init__(self , x , y  ):
        self.x = x # x-position
        self.y = y # y-postion
        self.id = 0 # vertices id 
        self.f_score = float('inf') # initialize as inifinity 
        self.g_score = float('inf') # initialize as inifinity 
        self.parent  = None
       
    def calcu_huristic(self,target):
        distance = sqrt( (self.x-target.x)**2 + (self.y-target.y)**2 )
        return distance 
    
    def get_distance(self,target):
        distance = sqrt( (self.x-target.x)**2 + (self.y-target.y)**2 )
        return distance 
    def find_nearest_node(self,nodes):
        min_distance = float('inf')
        nearest_node = None
        for node in nodes:
            distance = self.get_distance(node)
            if(distance < min_distance):
                nearest_node = node
                min_distance = distance
        return nearest_node
    def find_nodes_with_in_radius(self,nodes,radius):
        # print(nodes)
        nodes_with_in_radius =[]
        for node in nodes:
            distance = self.get_distance(node)
            # less than or equal to the radius and the node is not the same as the current node
            if(distance <= radius and self != node):
              nodes_with_in_radius.append(node)
        return nodes_with_in_radius
    def find_costopt_nearest_node(self,nodes ,radius):
        

        # filter a nodes with a given radius 
        nodes_with_in_radius = self.find_nodes_with_in_radius(nodes,radius)

        if(not nodes_with_in_radius): # return the nearest node 
            return self.find_nearest_node(nodes)
        else : 
            # find a node with minimum cost from start to the given node
            optimam_node = self.find_nearest_node(nodes) 
            min_cost = float("inf")
            # print(nodes_with_in_radius)

            for node in  nodes_with_in_radius:
                
                cost = node.g_score + self.get_distance(node)
                if(cost < min_cost):
                    optimam_node = node
                    min_cost = cost
            
            return optimam_node 
    def __str__(self):
        return str(self.id)
    # def __eq__(self,other):
    #     print("other" , other)
    #     return self.id == other.id
    
class RRT:
    def __init__(self,map,k,q,p,start,goal,is_RRT_star=False):
        
        self.map    = map
        self.k      = k
        self.q      = q
        self.p      = p 
        self.start  = start
        self.goal   = goal
        self.vertices = []
        self.edges = []
        self.vertices_star= []
        self.edges_star = []
        self.height ,self.width = map.shape[0] , map.shape[1]
        self.node_counter = 1
        self.path = []
        self.smoothed_path = []
        self.is_RRT_star = is_RRT_star # by deafault it is False we implement RRT
        self.radius = 10
            
    def Rand_Config(self):       
        probab = random()
        qrand = self.goal
      
        if(probab > self.p):          
           x = randint(0,self.map.shape[0])
           y = randint(0,self.map.shape[1])
           qrand = Node(x,y)

        return qrand
    
    def cost_optimal_parent(self,qnew , current_parent):
        # filter a nodes with a given radius 
        nodes_with_in_radius = qnew.find_nodes_with_in_radius(self.vertices,self.radius)
        current_cost = current_parent.g_score + qnew.get_distance(current_parent)
        if(not nodes_with_in_radius): # return the nearest node
            return current_parent
        else :
            for node in nodes_with_in_radius:
                new_node_cost = node.g_score + qnew.get_distance(node)
                if(new_node_cost < current_cost and self.is_segment_free(node,qnew)):
                    current_parent = node

            return current_parent
            

    def rewire(self,qnew):
        # filter a nodes with a given radius 
        nodes_with_in_radius = qnew.find_nodes_with_in_radius(self.vertices,self.radius)

        for node in  nodes_with_in_radius:
            new_node_cost = qnew.g_score + qnew.get_distance(node)      
            

            # "if its parent has optimal rewire cost discard considering child rewiring 
            # we are goint to deal with its parent in the nodes_with_in_radius
            # this advantageous if its parent has optimal rewire cost the parent of the child does not
            #  change 

            # if( node.parent != None and node.parent in nodes_with_in_radius ):
            #     new_parent_node_cost = qnew.g_score + qnew.get_distance(node.parent)
            #     # if the parent is feasible to be add continue add in the next step
            #     if(self.is_segment_free(node.parent,qnew) and new_parent_node_cost < node.parent.g_score): # check if parent node is obstacle free
            #          continue # Add the parent of the node not the child
            # if the new node cost is less than the old node cost and the segment is free
            if(new_node_cost < node.g_score and self.is_segment_free(node,qnew)):
                node.parent = qnew # update the parent of the node
                node.g_score = new_node_cost # update the g_score
                node.f_score = node.g_score + node.get_distance(self.goal) # update the f_score
            
    def Near_Vertices(self , qrand):
        qnear =  qrand.find_nearest_node(self.vertices )
        return qnear
    def New_Config(self , qnear , qrand):
        dir_vector = np.array([qrand.x -  qnear.x , qrand.y - qnear.y])
        length = qnear.get_distance(qrand)
        
        if(self.q > length):
            return qrand
        else:
   
            norm_vector = dir_vector/length
            qnew = np.array([qnear.x,qnear.y]) + norm_vector*self.q      
            qnew = Node(int(qnew[0]) , int(qnew[1]))
            return qnew
    
    def is_segment_free(self, q1, q2):
        """
        
        """
        y = q1.y
        x = q1.x                 # the line points
        dx = q2.x - q1.x
        dy = q2.y - q1.y

        if dy < 0:
            ystep = -1
            dy = -dy
        else:
            ystep = 1

        if dx < 0:
            xstep = -1
            dx = -dx
        else:
            xstep = 1

        ddy = 2 * dy                        # work with double values for full precision
        ddx = 2 * dx
        
        if ddx >= ddy:                    # first octant (0 <= slope <= 1)
            # compulsory initialization (even for errorprev, needed when dx==dy)
            error       = dx                # start in the middle of the square
            errorprev   = error

            for i in range(dx):
                # Do not use the first point (already done)
                x += xstep
                error += ddy
                if error > ddx:             # increment y if AFTER the middle ( > )
                    y       += ystep
                    error   -= ddx
                    # three cases (octant == right->right-top for directions below):
                    if error + errorprev < ddx:     # bottom square also
                        if self.map[x, y-ystep] == 1.:
                            return False
                    elif error + errorprev > ddx:   # left square also
                        if self.map[x-xstep, y] == 1.:
                            return False
                    else:                           # corner: bottom and left squares also
                        if self.map[x, y-ystep] == 1.:
                            return False
                        if self.map[x-xstep, y] == 1.:
                            return False

                if self.map[x, y] == 1.:
                    return False
                errorprev = error
        else:               # the same as above
            error       = dy
            errorprev   = error
            for i in range(dy):
                y       += ystep
                error   += ddx
                if error > ddy:
                    x       += xstep
                    error   -= ddy
                    if error + errorprev < ddy:
                        if self.map[x-xstep, y] == 1.:
                            return False
                    elif error + errorprev > ddy:
                        if self.map[x, y-ystep] == 1.:   
                            return False
                    else:
                        if self.map[x-xstep, y] == 1.:
                            return False
                        if self.map[x, y-ystep] == 1.:
                            return False

                if self.map[x, y] == 1.:     
                    return False
                errorprev = error

        return True

    def is_onboard( self , node ):       
        if( 0<=node.x <self.height and 0<= node.y < self.width):
            return True        
        else:
            return False   
    def isValid(self,node): 
        if self.is_onboard(node) and self.map[node.x,node.y] != 1 and  self.map[node.x,node.y] == 0 :
            return True
        return False
     # plotting
    def plot(self):
        x_path = [v.x for v in self.path ]
        y_path = [v.y for v in self.path ]
        x_spath = [v.x for v in self.smoothed_path ]
        y_spath = [v.y for v in self.smoothed_path ]
        plt.matshow(self.map) 
        for i,edge in enumerate(self.edges):
            p0x , p1x  =  [self.vertices[edge[0].id].x , self.vertices[edge[1].id].x] 
            p0y , p1y   = [self.vertices[edge[0].id].y , self.vertices[edge[1].id].y]

            plt.plot([edge[0].y,edge[1].y],[edge[0].x,edge[1].x],color='red',linestyle ='--')   
        
        # for i,v in enumerate(self.vertices):
        #     plt.text(v.y-0.05,v.x+0.08, str(i))
        plt.plot(self.start.y,self.start.x,color='green', marker = '*')
        plt.plot(self.goal.y,self.goal.x,color='green', marker = '*')
         
        plt.plot(y_path,x_path,color='green')
        plt.plot(y_spath,x_spath,color='yellow')
        plt.show()
 
    def reconstract_path(self):

        current = self.goal
        self.path = [current]
        while( current != self.start):
            current = current.parent
            self.path.append(current)
        path =[(n.x,n.y) for n in self.path]
        # print("path" , path)
        # print("path", self.path)
        self.path.reverse()
        path =[(n.x,n.y) for n in self.path]
        print("path" , path)
    def smooth_path(self):

        current = self.start
        self.smoothed_path.append(current)
      
        while(current != self.goal):
        
            i = len(self.path)-1
            while(current != self.path[i]):
                if(self.is_segment_free(current,self.path[i])):
                    current = self.path[i]
                    current.parent = self.smoothed_path[-1]
                    current.g_score = current.parent.g_score + current.get_distance(current.parent)
                    current.f_score = current.g_score + current.calcu_huristic(self.goal)
                    self.smoothed_path.append(current) # add the visibile node 
                    break
                elif(current  == self.path[i]):
                    self.smoothed_path.append(current)
                    current = current.parent # add the parent of the current node all are not visibile
                i -= 1
            

        # self.smoothed_path.reverse()
        path =[(n.x,n.y) for n in self.smoothed_path]
        # print("smooth_path:" , path)
        return self.smoothed_path
    def compute_path(self):
        self.vertices.append(self.start) # initialize the vertices list 
        self.start.g_score = 0
        self.start.f_score = self.start.g_score + self.start.calcu_huristic(self.goal)
        
        # plt.matshow(self.map)

        for k in range(self.k):
            qrand = self.Rand_Config()  
            # print("rand" ,qrand.x , qrand.y)
            
            while(not self.isValid(qrand)):
               qrand = self.Rand_Config()
               
            qnear = self.Near_Vertices(qrand)
            # print("qnear" ,qnear.x , qnear.y)
            qnew  = self.New_Config(qnear,qrand)

            # print("qnew" ,qnew.x , qnew.y)
            # print(self.is_segment_free())
            if( self.is_segment_free(qnear , qnew)):  
                # plt.scatter(qrand.y,qrand.x, color='blue')
                # plt.plot([qrand.y, qnear.y], [qrand.x, qnear.x], 'r-')
              
                # plt.plot([qnew.y, qnear.y], [qnew.x, qnear.x], 'b-')    
                # plt.show()
                
                qnear = self.cost_optimal_parent(qnew, qnear)
              
                self.vertices.append(qnew)
                self.edges.append((qnear,qnew))
                qnew.parent = qnear 
                qnew.id = self.node_counter 
                qnew.g_score = qnear.g_score + qnew.get_distance(qnear)
                qnew.f_score = qnear.g_score + qnew.calcu_huristic(self.goal)
                # Implement rewire here
                # print("qnew" ,qnew , qnew.x)
                if(self.is_RRT_star == True): # if is RRT star is true we implemet rewire function
                    self.rewire(qnew)
                if(qnew == self.goal):
                    
                    vertices =[n.id for n in self.vertices]
                    self.reconstract_path()            
                    return self.path
    
                self.node_counter +=1
            
                
def main():
    
    # Load grid map 
    image = Image.open("/home/elias/catkin_ws/src/turtlebot_online_path_planning/src/gridMap/map0.png").convert('L')
    grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1])/255 
    # binarize the image 
    grid_map[grid_map >  0.5] = 1
    grid_map[grid_map <= 0.5] = 0 
    # Invert colors to make 0 -> free and 1 -> occupied 
    grid_map = (grid_map * -1) + 1 # Show grid map 
    # print(grid_map)
    # plt.matshow(grid_map) 
    # plt.plot()gridMap/map0.png
    # plt.show()
    start = Node(10,10)   
    goal =  Node(90,70)
    K=1000 # number itrations 
    p=0.2 # probability of selecting goal
    delat_q = 5
    print(grid_map.shape)
    rrt = RRT(grid_map,K,delat_q,p,start,goal)
    path = rrt.compute_path() # compute rrt path 
    print("path_distance:", goal.f_score)
    rrt.smooth_path() # calling smooth_path function 
    print("smooth_path_dis:", goal.f_score)
    rrt.plot() # calling plot function 


    start = Node(10,10)   
    goal =  Node(90,70)
    rrt = RRT(grid_map,K,delat_q,p,start,goal,True)
    path = rrt.compute_path() # compute rrt path 
    print("rrt-star path_distance:", goal.f_score)
    rrt.smooth_path() # calling smooth_path function 
    print("rrt-star smooth_path_dis:", goal.f_score)
    rrt.plot() # calling plot function 
    
    
    
    # Rand Configuration 
    # Nearest Vertixes 
    #New Configurattion
main()
