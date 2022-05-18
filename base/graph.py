class Graph:
    def __init__(self, noOfVertices):
        self.noOfVertices = noOfVertices
        self.AdjList = {}

    def addVertex(self, v):
        self.AdjList[v] = []

    def addEdge(self, v, edge):
        self.AdjList[v].append(edge)

class Edge:
    def __init__(self, vertexA, vertexB):
        self.vertexA = vertexA
        self.vertexB = vertexB

    def __str__(self) -> str:
        return str(self.vertexA['number']) + " -> " + str(self.vertexB['number'])