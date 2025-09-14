import numpy as np 
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
class CSC:
   def __init__(self, xstart ,ystart,thetastart,xend,yend,thetaend,time,wheel_raduis,Mobile_robot_width,Mobile_robot_height,Map_width,Map_height):
   
   
    

            self.x_start=xstart
            self.y_start=ystart
            self.theta_start=thetastart

            self.xend=xend
            self.yend=yend
            self.thetaend=thetaend
            self.n=3
            self.T=time
            self.step=1
            self.Raduis=wheel_raduis
            self.W=Mobile_robot_width
            self.l=Mobile_robot_height
            self.Width=Map_width
            self.height=Map_height
            self.epsilon=int(self.T/(2*(self.n-2)+1))
            self.w=(2*math.pi)/self.epsilon
            self.Z1=np.zeros((4,1))
            self.Z2=np.zeros((4,1))
            self.Z3=np.zeros((4,1))
            self.Z1[0,0]=self.x_start
            self.Z2[0,0]=math.tan(self.theta_start)
            self.Z3[0,0]=self.y_start
            self.Z1[3,0]=self.xend
            self.Z2[3,0]=math.tan(self.thetaend)
            self.Z3[3,0]=self.yend
            Z1_first=self.x_start
            Z2_first=math.tan(self.theta_start)
            Z3_first=self.y_start
            Z1_end=self.xend
            Z2_end=math.tan(self.thetaend)
            Z3_end=self.yend
            self.X=np.zeros((int(self.T/self.step)+1,1))
            self.Y=np.zeros((int(self.T/self.step)+1,1))
            self.theta=np.zeros((int(self.T/self.step)+1,1))
            self.X[0,0]=self.x_start
            self.Y[0,0]=self.y_start
            self.theta[0,0]=self.theta_start
            self.calculate_C()
            self.calculate_time()
   def calculate_C(self): 

        c2=(self.Z1[3,0]-self.Z1[0,0])/self.epsilon
        c1=(self.Z3[3,0]-self.Z3[0,0]-self.epsilon*c2*self.Z2[0,0])/((self.epsilon**2)*c2)
        c3=(self.Z2[3,0]-self.epsilon*c1-self.Z2[0,0])/self.epsilon 
        self.C = np.array([[c1], [c2], [c3]]) 
 
   
   
   def calculate_z(self):
       
                
        for i in range(self.n):

            
            if i%2==0:
                self.Z1[i+1,0]=self.Z1[i,0]
                self.Z2[i+1,0]=self.C[i,0]*self.epsilon+self.Z2[i,0]
                self.Z3[i+1,0]=self.Z3[i,0]
            else:
                
                self.Z1[i+1,0]=self.C[i,0]*self.epsilon+self.Z1[i,0]
                self.Z2[i+1,0]=self.Z2[i,0]
                self.Z3[i+1,0]=self.C[i,0]*self.Z2[i,0]*self.epsilon+self.Z3[i,0]

   def calculate_time(self):
       
        t1 = np.arange(0, self.epsilon, self.step)
        t2 = np.arange(self.epsilon, 2*self.epsilon, self.step)
        t3 = np.arange(2*self.epsilon, 3*self.epsilon+self.step, self.step)  
        self.total_time=np.array([t1,t2,t3])
        print(len(self.total_time[1]))
 
   def calculate_V_cordinates(self,count,i,t,V1,V2):
        if i%2==0:
                    

            V1.append(0)
            
            V2.append(self.C[i,0]*(1-math.cos(self.w*t)))
            self.X[count,0]=self.Z1[i,0]
            self.Y[count,0]=self.Z3[i,0]
            self.theta[count,0]=(math.atan(self.C[i,0]*t-(self.C[i,0]/self.w)*math.sin(self.w*t)+self.Z2[i,0]-self.epsilon*i*self.C[i,0]))
        else:
            V2.append(0)
            V1.append(self.C[i,0]*(1-math.cos(self.w*t)))
            self.X[count,0]=self.C[i,0]*t-(self.C[i,0]/self.w)*math.sin(self.w*t)+self.Z1[i,0]-self.epsilon*self.C[i,0]
            self.Y[count,0]=self.Z2[i,0]*self.C[i,0]*t-self.Z2[i,0]*self.C[i,0]*math.sin(self.w*t)/self.w+self.Z3[i,0]-self.Z2[i,0]*self.C[i,0]*self.epsilon
            self.theta[count,0]=math.atan(self.Z2[i,0])
            
   def draw_obstacles(self,ax,list):
        for j in range(len(list)):
                    
                    
                    if j==0 or j==len(list)-1:
                        start_point=patches.Circle((self.x_start,self.y_start),radius=0.1,color='black')
                        ax.add_patch(start_point)
                        end_point=patches.Circle((self.xend,self.yend),radius=0.1,color='black')
                        ax.add_patch(end_point)
                    elif j%4==1:
                        
                        obb=patches.Polygon([(list[j].xv + self.W/2 +self.Raduis,list[j].yv -self.l/2 -self.Raduis ), (list[j+1].xv -self.W/2 -self.Raduis, list[j+1].yv-self.l/2 -self.Raduis), (list[j+2].xv-self.W/2 -self.Raduis, list[j+2].yv+self.l/2 +self.Raduis),(list[j+3].xv+self.W/2 +self.Raduis, list[j+3].yv+self.l/2 +self.Raduis) ], closed=True, edgecolor='green', facecolor='green')
                        ax.add_patch(obb)
   def draw_mobile_robot(self,count,ax)  :
                x1=self.X[count,0]+ -self.W/2*math.cos(self.theta[count,0])+self.l/2 *math.sin(self.theta[count,0])
                y1=self.Y[count,0]+ -self.W/2 *math.sin(self.theta[count,0])-self.l/2 *math.cos(self.theta[count,0])
                x2=self.X[count,0]+ -self.W/2*math.cos(self.theta[count,0])-self.l/2 *math.sin(self.theta[count,0])
                y2=self.Y[count,0]+ -self.W/2 *math.sin(self.theta[count,0])+self.l/2 *math.cos(self.theta[count,0])
                x3=self.X[count,0]+ self.W/2*math.cos(self.theta[count,0])-self.l/2 *math.sin(self.theta[count,0])
                y3=self.Y[count,0]+ +self.W/2 *math.sin(self.theta[count,0])+self.l/2 *math.cos(self.theta[count,0])
                x4=self.X[count,0]+ self.W/2*math.cos(self.theta[count,0])+self.l/2 *math.sin(self.theta[count,0])
                y4=self.Y[count,0]+ +self.W/2 *math.sin(self.theta[count,0])-self.l/2 *math.cos(self.theta[count,0])               
                vertices_of_mobilerobot = [(x1, y1), (x2, y2), (x3, y3),(x4, y4) ]
                mobilerobot = patches.Polygon(vertices_of_mobilerobot, closed=True, edgecolor='blue', facecolor='blue')
                ax.add_patch(mobilerobot)
                x1=self.X[count,0]+ -self.Raduis/2*math.cos(self.theta[count,0])+self.Raduis/2 *math.sin(self.theta[count,0])
                y1=self.Y[count,0]+ -self.Raduis/2 *math.sin(self.theta[count,0])-self.Raduis/2 *math.cos(self.theta[count,0])
                x2=self.X[count,0]+ -self.Raduis/2*math.cos(self.theta[count,0])-self.Raduis/2 *math.sin(self.theta[count,0])
                y2=self.Y[count,0]+ -self.Raduis/2 *math.sin(self.theta[count,0])+self.Raduis/2 *math.cos(self.theta[count,0])
                x3=self.X[count,0]+ self.Raduis/2*math.cos(self.theta[count,0])-self.Raduis/2 *math.sin(self.theta[count,0])
                y3=self.Y[count,0]+ +self.Raduis/2 *math.sin(self.theta[count,0])+self.Raduis/2 *math.cos(self.theta[count,0])
                x4=self.X[count,0]+ self.Raduis/2*math.cos(self.theta[count,0])+self.Raduis/2 *math.sin(self.theta[count,0])
                y4=self.Y[count,0]+ +self.Raduis/2 *math.sin(self.theta[count,0])-self.Raduis/2 *math.cos(self.theta[count,0])
                cc=(0.5*self.l+0.5*self.Raduis)*math.sin(self.theta[count,0])
                dd=(0.5*self.l+0.5*self.Raduis)*math.cos(self.theta[count,0])
                vertices_of_wheel = [(x1-cc, y1+dd), (x2-cc, y2+dd), (x3-cc, y3+dd),(x4-cc, y4+dd) ]
                wheel = patches.Polygon(vertices_of_wheel, closed=True, edgecolor='red', facecolor='red')
                ax.add_patch(wheel)
                vertices_of_wheel = [(x1+cc, y1-dd), (x2+cc, y2-dd), (x3+cc, y3-dd),(x4+cc, y4-dd) ]
                wheel = patches.Polygon(vertices_of_wheel, closed=True, edgecolor='red', facecolor='red')
                ax.add_patch(wheel)    
   def update(self,count,ax,V1,V2,list,i,t):
        ax.clear()
        ax.set_xlim((0,self.height))
        ax.set_ylim((0,self.Width))
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        
        self.draw_obstacles(ax,list)
        self.draw_mobile_robot(count,ax)
        
        
        return ax.patches
    

                                 
   def run_CSC(self,V1,V2,list):
        j=0
        countt=0
        self.calculate_z()
        for i in range(self.n):
                
            for t in self.total_time[i]:
                self.calculate_V_cordinates(countt,i,t,V1,V2)
                countt=countt+1
                
        fig, ax = plt.subplots()
               
                
        ani=FuncAnimation(fig,self.update,frames=len(self.X),fargs=(ax,V1,V2,list,i,t),interval=100,blit=True,repeat=False)
              
        
        plt.show() 
        
        
        
        
                
                
                