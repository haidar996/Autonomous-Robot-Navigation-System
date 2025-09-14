import math
class vertix:
    def __init__(self,x,y,asci):
        self.xv=float(x)
        self.yv=float(y)
        self.asci=asci
class dijkstra:
    def __init__(self,vertices_number,node_symbols,graph):
        self.V=vertices_number
        self.node_symbols=node_symbols
        self.graph=graph
    def min_weight(self,dist,visited):
        min=1e6
        min_node=None
        for node in range(self.V):
            if dist[node]>0 and visited[node]==False and min>dist[node]:
                min=dist[node]
                min_node=node
        return min_node           
    

    def print_shortest_path(self,goal,list):
        stack=[]
        pathx=[]
        pathy=[]
        path_theta=[]
        stack.append(self.node_symbols[goal])
        node=self.parent[goal]
        while node !=None:
            stack.append(self.node_symbols[node])
            node=self.parent[node]
           
        while len(stack)!=1:
            node=stack.pop()
            print(node,'->',end=" ")
            node_order=ord(node)-65
            pathx.append(list[node_order].xv)
            pathy.append(list[node_order].yv)
            path_theta.append(math.atan(list[node_order].yv/list[node_order].xv))
        node=stack.pop()
        print(node)
        
        pathx.append(list[-1].xv)
        pathy.append(list[-1].yv)
        path_theta.append(math.atan(list[-1].yv/list[-1].xv))

        
        return pathx,pathy,path_theta                
        

    def run_dijkstra(self,start_node):
        self.start_node=start_node
        dist=[1e6]*self.V
        dist[start_node] =0
        visited=[False]*self.V
        self.parent=[None]*self.V
        jumps=[0]*self.V    
        visited[start_node]=True
     
           
        node=start_node
        for i in range(self.V):

            for u in range(self.V):

                if self.graph[u][node]>0 and visited[u]==False and dist[u]>=dist[node]+self.graph[node][u] :

                    if jumps[node]+1>jumps[u] and dist[u]==dist[node]+self.graph[node][u]:
                        pass

                    else:

                        self.parent[u]=node
                        jumps[u]=jumps[node]+1
                    dist[u]=dist[node]+self.graph[node][u]
            nearest_node=self.min_weight(dist,visited)
            
            node=nearest_node
            if node==None:
                break
            visited[node]=True