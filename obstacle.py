
import math
import numpy as np
class vertix:
    def __init__(self,x,y,asci):
        self.xv=float(x)
        self.yv=float(y)
        self.asci=asci
class obstacle:
    
    def __init__(self,width,height,x,y,number_of_obstacle,Mobile_robot_width,wheel_raduis,Mobile_robot_height,list):
        self.width=float(width)
        self.height=float(height)
        self.x=float(x)
        self.y=float(y)
        self.number_of_obstacle=number_of_obstacle
        self.list=list
        self.vertices_of_obstacle(Mobile_robot_width,wheel_raduis,Mobile_robot_height)
    def vertices_of_obstacle(self,Mobile_robot_width,wheel_raduis,Mobile_robot_height):
        
        asci=66+4*(self.number_of_obstacle-1)
        
        
        
        self.list.append(vertix(self.x-self.width/2 -Mobile_robot_width/2 -wheel_raduis/2,self.y+self.height/2 +Mobile_robot_height/2 +wheel_raduis/2,asci))
        
        self.list.append(vertix(self.x+self.width/2 +Mobile_robot_width/2 +wheel_raduis/2,self.y+self.height/2 +Mobile_robot_height/2 +wheel_raduis/2 ,asci+1))
        
        self.list.append(vertix(self.x+self.width/2 + Mobile_robot_width/2 +wheel_raduis/2,self.y-self.height/2 -Mobile_robot_height/2 -wheel_raduis/2,asci+2))
        
        self.list.append(vertix(self.x-self.width/2 -Mobile_robot_width/2 -wheel_raduis/2,self.y-self.height/2 -Mobile_robot_height/2 -wheel_raduis/2,asci+3))
    def eucledian_weight(self,node1,node2):
         return  math.sqrt((node1.xv-node2.xv)**2 +(node1.yv-node2.yv)**2)
    def node_symbols(self):
        
        dict_node={}

        

        
        
          

        for i in range(len(self.list)):
            
            
            dict_node[i] = chr(self.list[i].asci)
          
        
        return dict_node    
    def map_obstacle2vertices(self,x_end,y_end) :
        self.list.append(vertix(x_end,y_end,90))
        

        graph=np.zeros((4*self.number_of_obstacle+2,4*self.number_of_obstacle+2 ))
        graph1=np.zeros((1,4*self.number_of_obstacle+2))
        for i in range(4*self.number_of_obstacle+2):
            if self.is_neighbour(self.list[i],self.list[4*self.number_of_obstacle+1]):
                  cost=self.eucledian_weight(self.list[i],self.list[4*self.number_of_obstacle+1])
                  graph1[0][i]=cost
            else :
                  graph1[0][i]=0
                       
            
            
        for i in range(4*self.number_of_obstacle+2):
            for j in range(4*self.number_of_obstacle+2):
                
                if self.is_neighbour(self.list[i],self.list[j]):
                   cost=self.eucledian_weight(self.list[i],self.list[j])
                   graph[i][j]=cost
                   graph[j][i]=cost
                else:
                   graph[i][j]=0
                   graph[j][i]=0
        print(self.number_of_obstacle*4+2)
        print(self.node_symbols()) 
        print(graph)          
        return graph,self.number_of_obstacle*4+2, self.node_symbols(),graph1          
    def is_neighbour(self,node1,node2):
        V=self.number_of_obstacle
        for t in np.arange(0, 1, 0.05):
            x_cordinate=(1-t)*node1.xv+t*node2.xv
            y_cordinate=(1-t)*node1.yv+t*node2.yv
            status=True
           
            for i in range(V):
               
                if x_cordinate>self.list[i*4+1].xv and x_cordinate<self.list[i*4+2].xv and y_cordinate<self.list[i*4+1].yv and   y_cordinate>self.list[i*4+3].yv:
                    status =False
                    break
            if status==False:
                break    
        return status 