import numpy as np
import math 
class optimized_APF:
    def __init__(self,p0,katt,krep,step_size,D_goal):
        self.p0=p0
        self.katt=katt
        self.krep=krep
        self.step_size=step_size
        self.D_goal=D_goal
    def run_optimized_APF(self,list,Mobile_robot_width,Mobile_robot_height,wheel_raduis):
         
         
         distance=[]
         
         
         
         
         Urep=np.zeros((int(len(list)-2/4),1))

         qrobotx=[]
         qroboty=[]
         theta=[]
         
         dob=np.zeros((int(len(list)-2/4),1))
         Frepx=np.zeros((int(len(list)-2/4),1))
         Frepy=np.zeros((int(len(list)-2/4),1))

         qrobotx.append(list[0].xv)
         qroboty.append(list[0].yv)
         d=math.sqrt((list[-1].xv-qrobotx[-1])**2 +(list[-1].yv-qroboty[-1])**2)
         distance.append(d)
         while d>self.D_goal:
            
            d=math.sqrt((list[-1].xv-qrobotx[-1])**2 +(list[-1].yv-qroboty[-1])**2)
            distance.append(d)
            Uatt=(0.5*self.katt*distance[-1]**2)
            
               
               
               
               
               
            for i in range(1,len(list)-1,4):
               obstacle_half_width=(-list[i].xv+list[i+1].xv)/2
               obstacle_half_height=(-list[i].yv+list[i+1].yv)/2
               centerx=(list[i].yv+list[i+1].yv)/2
               centery=(list[i].yv+list[i+2].yv)/2
               dob[i][0]=(math.sqrt((centerx-qrobotx[-1])**2 +(centery-qroboty[-1])**2))
               
                 
               PO=self.p0+math.sqrt(obstacle_half_width**2+obstacle_half_height**2)+Mobile_robot_width/2+wheel_raduis/2
               
               if dob[i][0]<PO:
               
                  Urep[i][0]=((0.5*self.krep*((1/dob[i][0])-(1/PO))**2))
                  Frepx[i][0]=((self.krep*((1/dob[i][0])-(1/PO))*(qrobotx[-1]-centerx)/(dob[i][0])**3))
                  Frepy[i][0]=((self.krep*((1/dob[i][0])-(1/PO))*(qroboty[-1]-centery)/(dob[i][0])**3))
               else:
                  Frepx[i][0]=0
                  Frepy[i][0]=0
                  Urep[i][0]=0
               


            Fattx=self.katt*(list[-1].xv-qrobotx[-1]) 
            Fatty=self.katt*(list[-1].yv-qroboty[-1])
            Fx=0
            Fy=0
            for i in range(1,len(list)-1,4):
               
               Fx=Fx+(Fattx+Frepx[i][0])
               Fy=Fy+(Fatty+Frepy[i][0])

            th=math.atan2(Fx,Fy)
            qrobotx.append(qrobotx[-1]+self.step_size*Fx)
            qroboty.append(qroboty[-1]+self.step_size*Fy)
            theta.append(th*180/math.pi)
            
         return qrobotx,qroboty ,theta
