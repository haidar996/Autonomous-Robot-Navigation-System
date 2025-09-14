
import numpy as np
import math 
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import tkinter as tk
import time
from path_planning_part.vertex import vertix
from path_planning_part.APF import APF
from path_planning_part.optimized_APF import optimized_APF
from path_planning_part.A_star import A_star
from path_planning_part.dijkstra import dijkstra

from path_planning_part.obstacle import obstacle

from Control_part.CSC import CSC




def obstacles_interception(number_of_obstacle):
    while True:
      


      x_obstacle=float(input("x_obstacle"))
      y_obstacle=float(input("y_obstacle"))
      obstacle_width=float(input("obstacle_width"))
      obstacle_height=float(input("obstacle_height"))
      obstacle_interruption=False
      
      for t in np.arange(0,1,0.05):
        x_ob_test_1=(1-t)*(x_obstacle-obstacle_width/2)+t*(x_obstacle+obstacle_width/2)
        x_ob_test_2=(1-t)*(x_obstacle-obstacle_width/2)+t*(x_obstacle+obstacle_width/2)
        x_ob_test_3=x_obstacle-obstacle_width/2
        x_ob_test_4=x_obstacle+obstacle_width/2
        y_ob_test_1=y_obstacle+obstacle_height/2
        y_ob_test_2=y_obstacle-obstacle_height/2
        y_ob_test_3=(1-t)*(y_obstacle-obstacle_height/2)+t*(y_obstacle+obstacle_height/2)
        y_ob_test_4=(1-t)*(y_obstacle-obstacle_height/2)+t*(y_obstacle+obstacle_height/2)

        for i in range(number_of_obstacle):
            first_condition=x_ob_test_1 >list[i*4].xv and  x_ob_test_1 <list[i*4+1].xv and y_ob_test_1 <list[i*4].yv and y_ob_test_1 >list[i*4+2].yv
            second_condition=x_ob_test_2 >list[i*4].xv and  x_ob_test_2 <list[i*4+1].xv and y_ob_test_2 <list[i*4].yv and y_ob_test_2 >list[i*4+2].yv
            third_condition=x_ob_test_3 >list[i*4].xv and  x_ob_test_3 <list[i*4+1].xv and y_ob_test_3 <list[i*4].yv and y_ob_test_3 >list[i*4+2].yv
            fourth_condition=x_ob_test_4 >list[i*4].xv and  x_ob_test_4 <list[i*4+1].xv and y_ob_test_4 <list[i*4].yv and y_ob_test_4 >list[i*4+2].yv
            fifth_condition=x_obstacle-obstacle_width/2 <list[i*4].xv and  x_obstacle+obstacle_width/2 >list[i*4+1].xv and y_obstacle+obstacle_height/2 >list[i*4].yv and y_obstacle-obstacle_height/2 <list[i*4+2].yv
            
            
            
            if  first_condition or second_condition or third_condition or fourth_condition or fifth_condition :
               obstacle_interruption=True
               break
        if obstacle_interruption==True:
                break 
      if obstacle_interruption==False:
                
            
             break
      else:
            print("error!  obstacles interption!")  
            list1.append(x_obstacle) 
            list1.append(y_obstacle)
            list1.append(obstacle_width)
            list1.append(obstacle_height)    
    return obstacle_interruption,x_obstacle,y_obstacle,obstacle_width,obstacle_height
def control1(pathx,pathy,path_theta,time,V1,V2):
    j=0


    for i in range(len(pathx)-1):
        

        x_startt=pathx[i-j]
        x_endd=pathx[i+1]
        y_startt=pathy[i-j]
        y_endd=pathy[i+1]
        theta_startt=path_theta[i-j]
        theta_endd=path_theta[i+1]
        

        

        if x_endd-x_startt<0.1 and y_endd-y_startt<0.1:
            j=j+1
            
        else:


            cosine_swtich=CSC(x_startt ,y_startt,theta_startt,x_endd,y_endd,theta_endd,time,wheel_raduis,Mobile_robot_width,Mobile_robot_height,Map_width,Map_height) 
            cosine_swtich.run_CSC(V1,V2,list)
            j=0
           
def start_end_obstacle_interception(x_end,y_end,x_start,y_start):
        obstacle_mobilerobot_intereption=False

        for i in range(number_of_obstacle):
            
            
                
                for t in np.arange(0,2*math.pi,0.05):

                    
                    if Mobile_robot_width>Mobile_robot_height:
                        r=Mobile_robot_width/2+wheel_raduis
                    else:
                        r=Mobile_robot_height/2+wheel_raduis
                    x1=r*math.cos(t)+x_end
                    y1=r*math.sin(t)+y_end
                    
                    if (x1>list[i*4+1].xv and  x1 <list[i*4+2].xv and y1 <list[i*4+1].yv and y1 >list[i*4+3].yv)or (x_end>list[i*4+1].xv and  x_end <list[i*4+2].xv and y_end <list[i*4+1].yv and y_end >list[i*4+3].yv) or (math.sqrt((x1-x_start)**2 +(y1-y_start)**2)<r) :
                        print("error! obstacle robot intereption")
                        obstacle_mobilerobot_intereption=True
                        break
                
                if obstacle_mobilerobot_intereption==True:
                    break
        return   obstacle_mobilerobot_intereption
def path_planning(graph,number_of_vertices,node_symbols,graph1):
    if algorithm=='dijkstra':
    
  
        run = dijkstra(number_of_vertices, node_symbols,graph)
        run.run_dijkstra(0)
        pathx,pathy,path_theta=run.print_shortest_path(number_of_vertices-1,list)
  
  
    elif algorithm=='optimized_APF':
        run=optimized_APF(1,0.2,10,0.05,0.05)
        pathx,pathy,path_theta=run.run_optimized_APF(list,Mobile_robot_width,Mobile_robot_height,wheel_raduis)
       
    
    elif algorithm=='A_star':
        run = A_star(number_of_vertices, node_symbols,graph,graph1)
        run.run_A_star(0)
        pathx,pathy,path_theta=run.print_shortest_path(number_of_vertices-1,list)
    return pathx,pathy,path_theta    
def mobile_robot_obstacle_interception(x_start,y_start):
        obstacle_mobilerobot_intereption=False

        for i in range(number_of_obstacle):
            
            
                
                for t in np.arange(0,2*math.pi,0.05):

                    
                    if Mobile_robot_width>Mobile_robot_height:
                        r=Mobile_robot_width/2+wheel_raduis
                    else:
                        r=Mobile_robot_height/2+wheel_raduis
                    x1=r*math.cos(t)+x_start
                    y1=r*math.sin(t)+y_start
                    if x1>list[i*4+1].xv and  x1 <list[i*4+2].xv and y1 <list[i*4+1].yv and y1 >list[i*4+3].yv:
                        print("error! obstacle robot intereption")
                        obstacle_mobilerobot_intereption=True
                        break
                
                    if obstacle_mobilerobot_intereption==True:
                       break
        return   obstacle_mobilerobot_intereption  
def mobile_robot_cordinates():
     while True:
            
            
            x_start=float(input("X_start:="))
            y_start=float(input("y_start:="))
            theta_start=float(input("theta_start(degrees):="))
            obstacle_mobilerobot_intereption=mobile_robot_obstacle_interception(x_start,y_start)
            
            

            
            if  obstacle_mobilerobot_intereption==False:
                break
     list.append(vertix(x_start,y_start,65))
     rep=list[len(list)-1]

     j=len(list)-1
     while j>0:
        list[j]=list[j-1]

        j=j-1
     list[0]=rep            

                

     while True:
    
            
        x_end=float(input("X_end:="))
        y_end=float(input("y_end:="))
        theta_end=float(input("theta_end(degrees):="))
        start_end_intereption=start_end_obstacle_interception(x_end,y_end,x_start,y_start)
        
        if start_end_intereption==False:
            break
     return x_start,y_start,theta_start,x_end,y_end,theta_end    



list=[]
list1=[]
Map_height=float(input("Map height:="))
Map_width=float(input("Map width:="))
Mobile_robot_width=float(input("Mobile_robot_width:=") )
Mobile_robot_height=float(input("Mobile_robot_height:=")) 
wheel_raduis=float(input("wheel_raduis:="))
number_of_obstacle=0
answer=input("add obstacles? [y/n]")

obstacles_list=[]

    

if answer=='y':
        while answer !='n':
            

            obstacle_interruption,x_obstacle,y_obstacle,obstacle_width,obstacle_height=obstacles_interception(number_of_obstacle)
               
            
            number_of_obstacle=number_of_obstacle+1
            print(number_of_obstacle)
            ob=obstacle(obstacle_width,obstacle_height,x_obstacle,y_obstacle,number_of_obstacle,Mobile_robot_width,wheel_raduis,Mobile_robot_height,list)
           
            answer=input("more obstacles? [y/n]")
        x_start,y_start,theta_start,x_end,y_end,theta_end=mobile_robot_cordinates() 


     
    

    

    

          
            

                


            
            
        
        

                    
                
                
            

        Time=float(input("Time:"))
        algorithm=input("select the algorithm ")
        graph,number_of_vertices,node_symbols,graph1=ob.map_obstacle2vertices(x_end,y_end)
        pathx,pathy,path_theta=path_planning(graph,number_of_vertices,node_symbols,graph1)
        
        timebetweentwonodes=Time/(len(pathx)-1) 
        print(timebetweentwonodes)
        path_theta[0]=theta_start*math.pi/180
        path_theta[-1]=theta_end*math.pi/180
        V1=[]
        V2=[]
        print(pathx)
        
        



        control1(pathx,pathy,path_theta,timebetweentwonodes,V1,V2)
else:
    Time=float(input("Time:"))
    x_start,y_start,theta_start,x_end,y_end,theta_end=mobile_robot_cordinates()  
    V1=[]
    V2=[]
    cosine_swtich=CSC(x_start ,y_start,theta_start,x_end,y_end,theta_end,Time,wheel_raduis,Mobile_robot_width,Mobile_robot_height,Map_width,Map_height) 
    cosine_swtich.run_CSC(V1,V2,list) 

