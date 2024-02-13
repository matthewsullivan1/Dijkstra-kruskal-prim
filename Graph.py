import sys
from Heap import Heap

class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for _ in range(vertices)] for _ in range(vertices)]

    def add_edge(self, u, v, w):
        self.graph[u][v] = w
        self.graph[v][u] = w

    def dijkstra(self, src):
        # Stores shortest distance.
        dist = [sys.maxsize] * self.V
        # Shortest distance to the same node is 0.
        dist[src] = 0
        # Unvisited nodes
        marked = [False] * self.V


        for i in range(self.V):
            current = self.min_index(marked, dist)
            marked[current] = True

            for j in range(self.V):
                if self.graph[current][j] > 0 and marked[j] == False and dist[j] > dist[current] + self.graph[current][j]:
                    dist[j] = dist[current] + self.graph[current][j]

        # You have to call print_solution by passing dist.
        # In this way everyone's output would be standardized.
        self.print_dijkstra(dist)

    # finds the lowest weight vertice to visit next (if any)
    def min_index(self, marked, dist):
        min = sys.maxsize

        #marked = visited / unvisited nodes
        #dist = weights
        for vertice in range(self.V):
            if marked[vertice] == False and dist[vertice] < min:
                min_index = vertice
                min = dist[vertice]
        return min_index
    
    def print_dijkstra(self, dist):
        print("Vertex \t Distance from Source")
        for node in range(self.V):
            print(f"{node} \t->\t {dist[node]}")

    def prim(self):
        # Store the resulting graph.
        # where result[i] keeps the source vertex.
        # See the example output for expected result.
        result = [None] * self.V 
        min_weights = [sys.maxsize] * self.V
        marked = [False] * self.V
        min_weights[0] = 0

        for i in range(self.V):

            min = sys.maxsize
            for j in range(self.V):
                if min_weights[j] < min and marked[j] == False:
                    min = min_weights[j]
                    min_index = j
            marked[min_index] = True
 
            for j in range(self.V):
                if self.graph[min_index][j] > 0 and marked[j] == False and min_weights[j] > self.graph[min_index][j]:
                    min_weights[j] = self.graph[min_index][j]
                    result[j] = min_index
        
        # You have to call print_solution by passing the output graph.
        # In this way everyone's output would be standardized.
        self.print_prim(result)

    def print_prim(self, result):
        print("Edge \t Weight")
        for i in range(1, self.V):
            print(f"{result[i]} - {i} \t {self.graph[i][result[i]]}")

    def kruskal_mst(self):
        mst = []
        parent = [-1] * self.V
        edges = self.get_edges()

        '''
        sort all edges by their weight in ascending order

        iterate through sets
            pick the edge with the smallest weight
            if it forms a cycle, skip
            repeat until there is a connected tree with no missing nodes 
        '''
        def find_parent(u):
            if parent[u] == -1:
                return u
            return find_parent(parent[u])
        
        for edge in edges:
            u = edge[0] #v1
            v = edge[1] #v2

            u_parent = find_parent(u)
            v_parent = find_parent(v)

            # check for cycle 
            if u_parent != v_parent:
                mst.append(edge)
                parent[u_parent] = v_parent
            
        # Similar to the previous e.g. print your
        # resulting graph.
        self.print_kruskal(mst)



    def get_edges(self):
        edge_weight = []
        for i in range(self.V):
            for j in range(i + 1, self.V):
                if self.graph[i][j] != 0:
                    edge_weight.append([i, j, self.graph[i][j]])

        edge_weight = sorted(edge_weight, key=lambda x: x[2])
        return edge_weight

    def print_kruskal(self, result):
        print("Edge \t Weight")
        # Note that the below code is slightly different than the Prim's.
        # You can change this print code according to your choice, but
        # you have to display your graph in (vertex->vertex weight) format.
        for edge in result:
            print(f"{edge[0]} -> {edge[1]} \t {edge[2]}")





if __name__ == '__main__':
    # Create a graph with 21 vertices.
    graph = Graph(9)

    # Add edges and their weights.
    graph.add_edge(0, 1, 4)
    graph.add_edge(1, 2, 8)
    graph.add_edge(2, 3, 7)
    graph.add_edge(3, 4, 9)
    graph.add_edge(4, 5, 10)
    graph.add_edge(5, 6, 2)
    graph.add_edge(6, 7, 1)
    graph.add_edge(7, 0, 8)
    graph.add_edge(7, 1, 11)
    graph.add_edge(7, 8, 7)
    graph.add_edge(8, 2, 2)
    graph.add_edge(6, 8, 6)
    graph.add_edge(5, 2, 4)
    graph.add_edge(5, 3, 14)
    #graph.dijkstra(0)
    #graph.prim()

    graph.kruskal_mst()
