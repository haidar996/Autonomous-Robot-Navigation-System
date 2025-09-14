import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 
x=0
y=0
z=0
def call_back(data):
       global x,y,z
       x=data.pose.pose.position.x  
       y=data.pose.pose.position.y
       z=data.pose.pose.orientation.z
       #print (x)
       #print(y)
rospy.init_node("control_node",anonymous=True)
rospy.Subscriber("odom",Odometry,call_back)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
r=rospy.Rate(10)
     

typ = rospy.get_param('type')
x = rospy.get_param('init_x')
y = rospy.get_param('init_y')
ln = rospy.get_param('init_ln')
w= rospy.get_param('init_w')
source= rospy.get_param('src')
goal=rospy.get_param('goal')
center=[]
side=[]
goal[0]-=source[0]
goal[1]-=source[1]
for i in range(len(x)):
      center.append ([x[i]-source[0],y[i]-source[1]])
      side.append([ln[i],w[i]])
#print (center)
source=[0,0]
def create_edges(source, goal, center, side):
    num = 4*len(center) + 2
    edges = np.arange(num)
    coordinates = []
    coordinates.append(source)
    for i in range(len(center)):
        coordinates.append((center[i][0]-0.5*side[i][0],center[i][1]+0.5*side[i][1]))
        coordinates.append((center[i][0]+0.5*side[i][0],center[i][1]+0.5*side[i][1]))
        coordinates.append((center[i][0]-0.5*side[i][0],center[i][1]-0.5*side[i][1]))
        coordinates.append((center[i][0]+0.5*side[i][0],center[i][1]-0.5*side[i][1]))
    coordinates.append(goal)

    return edges,coordinates

def create_edges1(source, goal, center, side):
    num = 4*len(center) + 2
    edges = np.arange(num)
    coordinates = []
    coordinates.append(source)
    for i in range(len(center)):
        coordinates.append((center[i][0]-0.5*side[i][0]-0.25,center[i][1]+0.5*side[i][1]+0.25))
        coordinates.append((center[i][0]+0.5*side[i][0]+0.25,center[i][1]+0.5*side[i][1]+0.25))
        coordinates.append((center[i][0]-0.5*side[i][0]-0.25,center[i][1]-0.5*side[i][1]-0.25))
        coordinates.append((center[i][0]+0.5*side[i][0]+0.25,center[i][1]-0.5*side[i][1]-0.25))
    coordinates.append(goal)

    return coordinates

def point_links(point, coordinates, rectangles):
    links = []
    for i in range(len(coordinates)):
        if(i==point):
            links.append(0)
            continue
        t = 0
        dt = 0.005
        x1 = coordinates[point][0]
        y1 = coordinates[point][1]
        x2 = coordinates[i][0]
        y2 = coordinates[i][1]
        flag = 0
        while(t<=1):
            x = (1-t)*x1 + t*x2
            y = (1-t)*y1 + t*y2
            t+=dt
            t = round(t,3)
            flag = 0
            j=0
            while(j<len(rectangles)):
                if x>rectangles[j][0] and x<rectangles[j+1][0] and y>rectangles[j][1] and y<rectangles[j+1][1]:
                    flag = 1
                    break
                j += 2

            if flag==1:
                break
        if flag==0:
            dist = np.sqrt((coordinates[point][0] - coordinates[i][0]) ** 2 + (coordinates[point][1] - coordinates[i][1]) ** 2)
            links.append(dist)
        else:
            links.append(0)
    return links





class TwoWheeledRobot:
    def __init__(self, wheel_radius=1, robot_width=5):
        self.wheel_radius = wheel_radius
        self.robot_width = robot_width

    def update_position(self, time, epsilon, omega, c1, c2, c3, z1_0, z1_T, z2_0, z2_T, z3_0, z3_T):

        if(time<=epsilon):
            t = time
            self.x = z1_0
            self.y = z3_0
            self.theta = np.arctan(c1*t - c1*np.sin(omega*t)/omega + z2_0)
            self.v1 = 0
            self.v2 = c1*(1-np.cos(omega*t))
        elif(time<=2*epsilon):
            t = time-epsilon
            self.x = c2*t - c2*np.sin(omega * t)/omega + z1_0
            self.y = (c1*epsilon + z2_0)*(c2*t - c2*np.sin(omega*t)/omega) + z3_0
            self.v1 = c2 * (1 - np.cos(omega * t))
            self.v2 = 0
        else:
            t = time - 2 * epsilon
            self.theta = np.arctan(c3 * t - c3 * np.sin(omega * t) / omega + c1 * epsilon + z2_0)
            self.v1 = 0
            self.v2 = c3 * (1 - np.cos(omega * t))


    def get_position(self):
        return self.x, self.y, self.theta

    def simulate_robot_movement(self, path_points, duration, center, side):
        dt = 0.1
        time = 0
        positions = []
        v1 = []
        v2 = []
        time_vals = []

        epsilon = duration/3
        omega = 2*np.pi/epsilon
        for i in range(len(path_points)-1 ):
            z1_0 = x
            
            z2_0=np.tan(0)
            z3_0 = y
            z1_T = path_points[i + 1][0]
            z2_T = np.tan(0)
            z3_T = path_points[i + 1][1]
            c2 = (z1_T - z1_0) / epsilon
            if (c2==0):
               c1=10
               c2=10
            else:
               c1 = ((z3_T - z3_0) / (epsilon * c2) - z2_0) / epsilon
            c3 = (z2_T - z2_0) / epsilon - c1
            msg=Twist()
            #print("###################################################################")
            while(time <= (i + 1) * duration):
               self.update_position(time - i * duration, epsilon, omega, c1, c2, c3, z1_0, z1_T, z2_0, z2_T, z3_0, z3_T)
               time += dt
               positions.append(self.get_position())
               v1.append(self.v1)
               v2.append(self.v2)
               time_vals.append(time)
               u1 = self.v1/np.cos(self.theta)
               u2 = self.v2*np.cos(self.theta)*np.cos(self.theta)
               print (u1)
               msg.linear.x=u1
               msg.angular.z=u2
               pub.publish(msg)
               r.sleep()
            x_vals = [pos[0] for pos in positions]
            y_vals = [pos[1] for pos in positions]
            headings = [pos[2] for pos in positions]
            u1 = np.array(v1)/np.cos(headings)
            u2 = np.array(v2)*np.cos(headings)*np.cos(headings)
            thetaRdot = u1/self.wheel_radius + (self.robot_width/(2*self.wheel_radius))*u2
            thetaLdot = 2*u1 / self.wheel_radius - thetaRdot

        

        return thetaRdot,thetaLdot

   

#########################################################
class Graph():
    def __init__(self, vertices, nodes_symbols):
        self.V = vertices
        self.nodes_symbols = nodes_symbols

    def printSolution(self, dist):
        for node in range(self.V):
            print(f'{node}:   {dist[node]}')

    def printShortestPath(self, goal):
        shortest_path = [goal];
        parent = self.parents
        print(self.nodes_symbols[goal], end="<-")
        for i in range(self.V):
            if parent[goal] == self.src:
                shortest_path.append(parent[goal])
                break
            else:
                print(self.nodes_symbols[parent[goal]], end="<-")
                shortest_path.append(parent[goal])

            goal = parent[goal]
            shortest_path = list(reversed(shortest_path))
        return shortest_path

    def minDistance(self, dist, visited):

        min = 1e6
        min_node = None

        for v in range(self.V):
            if dist[v] < min and visited[v] == False:
                min = dist[v]
                min_node = v

        return min_node

    def dijkstra(self, src):
        self.src = src
        dist = [1e6] * self.V
        dist[src] = 0
        visited = [False] * self.V
        parent = [None] * self.V

        for node in range(self.V):

            u = self.minDistance(dist, visited)
            visited[u] = True

            for v in range(self.V):
                if (self.graph[u][v] > 0 and visited[v] == False) and (dist[v] > dist[u] + self.graph[u][v]):
                    dist[v] = dist[u] + self.graph[u][v]
                    parent[v] = u

        self.parents = parent

    def heuristic(self, node, goal):
        return math.sqrt((self.h[node][0] - self.h[goal][0]) ** 2 + (self.h[node][1] - self.h[goal][1]) ** 2)

    def a_star(self, src, goal):
        self.src = src
        dist = [1e6] * self.V
        dist[src] = 0
        score = [1e6] * self.V
        score[src] = self.heuristic(src, goal)
        visited = [False] * self.V
        parent = [None] * self.V

        for node in range(self.V):

            u = self.minDistance(score, visited)
            visited[u] = True

            for v in range(self.V):
                if (self.graph[u][v] > 0 and visited[v] == False) and (
                        score[v] > dist[u] + self.graph[u][v] + self.heuristic(v, goal)):
                    dist[v] = dist[u] + self.graph[u][v]
                    score[v] = dist[u] + self.graph[u][v] + self.heuristic(v, goal)
                    parent[v] = u

        self.parents = parent


#############################################################
#start_time=rospy.get_time()
graph = []
edges,coordinates = create_edges(source, goal, center, side)
coordinates_new=create_edges1(source, goal, center, side)
rectangles = []
for i in range(len(center)):
    rectangles.append((center[i][0] - 0.5 * side[i][0]-0.25, center[i][1] - 0.5 * side[i][1]-0.25))
    rectangles.append((center[i][0] + 0.5 * side[i][0]+0.25, center[i][1] + 0.5 * side[i][1]+0.25))

nodes_symbols={}
for i in edges:
    links = point_links(i, coordinates_new, rectangles)
    graph.append(links)
    if(i==edges[-1]):
        nodes_symbols[i] = 'Goal'
    elif (i == edges[0]):
        nodes_symbols[i] = 'Source'
    else:
        nodes_symbols[i] = chr(i+64)

g = Graph(len(edges), nodes_symbols)
g.graph = graph
g.h = coordinates_new
g.a_star(edges[0], edges[-1])
if (typ=="dijkstra"):
	g.dijkstra(edges[0])
#print("The first rectangle has edges of names A-B-C-D and the second")
#print("one has edges of names E-F-G-H and so on....")
shortest_path = g.printShortestPath(edges[-1])
path_points = [g.h[i] for i in shortest_path]
#print(path_points)
robot = TwoWheeledRobot()
robot.simulate_robot_movement(path_points, 20, center, side)
end_time=rospy.get_time()
simulation_time=end_time-start_time
error=math.sqrt((x-goal[0])**2+(y-goal[1])**2)
dist=math.sqrt((source[0]-goal[0])**2+(source[1]-goal[1])**2)
per_error=error*100/dist
print ("the final x_pos")
print (x)
print ("the final y_pos")
print (y)
print ("the simulation time is:")
print (simulation_time)
print ("the percentage error is:")
print(per_error)
