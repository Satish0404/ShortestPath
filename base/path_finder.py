from json.encoder import INFINITY
from math import sqrt
import math
import xml.etree.ElementTree as ET
from base.avl_tree import AVLTree

from base.graph import Edge, Graph
from queue import PriorityQueue




class PathFinder:
    def __init__(self, pointsXml, polygonsXml):
        self.pointsXml = pointsXml
        self.polygonsXml = polygonsXml
        self.points = []
        self.polygons = []
        self.noOfVertices = 0
        self.graph = None
        self.allPoints = []

        self.fixedPoint = None
        self.prevVertex  = None
        self.prevVertexVisible = False
        self.maxPoint = None
        self.avlTree = AVLTree(self.compareEdges)
        self.lastMin = None

        self.loadData()


    def loadData(self):
        pointsTree = ET.parse(self.pointsXml)
        pointsRoot = pointsTree.getroot()

        for element in pointsRoot:
            element.attrib['x'] = int(element.attrib['x'])
            element.attrib['y'] = int(element.attrib['y'])
            self.points.append(element.attrib)


        polygonTree = ET.parse(self.polygonsXml)
        polygonRoot = polygonTree.getroot()

        self.maxPoint = {'x': 0,'y': 0}

        self.noOfVertices = 2
        for element in polygonRoot:
            points = []
            for point in element:
                self.noOfVertices += 1
                point.attrib['x'] = int(point.attrib['x'])
                point.attrib['y'] = int(point.attrib['y'])
                points.append(point.attrib)
            self.polygons.append(points)

        self.graph = Graph(self.noOfVertices)
        vertexCount = 2

        self.allPoints.append({"number": 0, "point" : self.points[0], "incidentVertices": [], "polygon" : -1})
        self.allPoints.append({"number": 1, "point" : self.points[1], "incidentVertices": [], "polygon" : -2})

        self.graph.addVertex(0)
        self.graph.addVertex(1)

        countPolygons = 0
        for polygonPoints in self.polygons:

            helpVertexCount = vertexCount

            for point in polygonPoints:
                self.allPoints.append({"number": vertexCount, 
                "point" : point,
                "incidentVertices": [], 
                "polygon" : countPolygons})

                vertexCount += 1
                self.graph.addVertex(vertexCount)

            j = 0
            for point in polygonPoints:
                if j == 0:
                    self.allPoints[helpVertexCount]['incidentVertices'].append(helpVertexCount + 1)
                    self.allPoints[helpVertexCount]['incidentVertices'].append(helpVertexCount - 1 + len(points))
                    helpVertexCount += 1
                elif j == len(polygonPoints) - 1:
                    self.allPoints[helpVertexCount]['incidentVertices'].append(helpVertexCount + 1 - len(points))
                    self.allPoints[helpVertexCount]['incidentVertices'].append(helpVertexCount - 1)
                else:
                    self.allPoints[helpVertexCount]['incidentVertices'].append(helpVertexCount + 1)
                    self.allPoints[helpVertexCount]['incidentVertices'].append(helpVertexCount - 1)
                    helpVertexCount += 1
                j += 1


            countPolygons += 1





    def calculateVisibiltyGraph(self):
        for vertex  in self.allPoints:
            visibleFromVertex  = self.visibleVertices(vertex)
            for visibleVertex in visibleFromVertex:
                if (vertex['number'] != visibleVertex['number']):
                    self.graph.addEdge(visibleVertex['number'], { 'otherVertex': visibleVertex['number'], 'distance': 0 })

            print(visibleFromVertex)






    def visibleVertices(self, currentVertex):
        allOtherPoints = self.allPoints.copy()

        for otherPoint in allOtherPoints:
            otherPoint['distance'] = self.distance(currentVertex["point"], otherPoint["point"])
            otherPoint['angle'] = math.atan2(otherPoint["point"]['x'] - currentVertex["point"]['x'], otherPoint["point"]['y'] - currentVertex["point"]['y'])*(180/math.pi)
            

            if (otherPoint['angle'] < 0):
                otherPoint['angle'] = otherPoint['angle'] + 360


        allOtherPoints.sort(key=lambda d: d['angle'])

        res = []
        self.avlTree = AVLTree(self.compareEdges)

        self.fixedPoint  = currentVertex
        self.prevVertex = None


        for onePoint  in self.allPoints:
            if onePoint['number'] == 0 or onePoint['number'] == 1 or onePoint['number'] == currentVertex['number'] or self.allPoints[onePoint['incidentVertices'][0]] == currentVertex['number']:
                continue

            self.maxPoint['x'] = 50000
            self.maxPoint['y'] = 0

            if (self.lineIntersection(0, 0, self.maxPoint['x'], self.maxPoint['y'], onePoint["point"]['x'] - currentVertex["point"]['x'], currentVertex["point"]['y'] - onePoint["point"]['y'], self.allPoints[onePoint['incidentVertices'][0]]['point']['x'] - currentVertex["point"]['x'], currentVertex["point"]['y'] - self.allPoints[onePoint['incidentVertices'][0]]['point']['y'])):
                edge = Edge(onePoint, self.allPoints[onePoint['incidentVertices'][0]])
                self.avlTree.insert(edge)

        for i in range(0,len(allOtherPoints)):
            if (allOtherPoints[i]['number'] != currentVertex['number']):
                if (allOtherPoints[i]['point']['x'] - currentVertex['point']['x'] != 0):
                    
                    try:
                        slope = (-(currentVertex['point']['y'] - currentVertex['point']['y'])) / -((currentVertex['point']['x'] - currentVertex['point']['x']))
                    except:
                        if (-(currentVertex['point']['y'] - currentVertex['point']['y'])) > 0:
                            slope = float('inf')
                        else:
                            slope = -float('inf')


                    if (abs(slope) > 1):
                        if (currentVertex['point']['y'] - allOtherPoints[i]['point']['y'] > 0):
                            self.maxPoint['y'] = 50000
                            self.maxPoint['x'] = 50000 / slope
                        else:
                            self.maxPoint['y'] = -50000
                            self.maxPoint['x'] = -50000 / slope
                    else:
                        if (allOtherPoints[i]['point']['x'] - currentVertex['point']['x'] > 0):
                            self.maxPoint['x'] = 50000
                            self.maxPoint['y'] = slope * 50000
                        else:
                            self.maxPoint['x'] = -50000
                            self.maxPoint['y'] = slope * -50000

                else:
                    if (currentVertex['point']['y'] - allOtherPoints[i]['point']['y'] > 0):
                        self.maxPoint['x'] = 0
                        self.maxPoint['y'] = 50000
                    else:
                        self.maxPoint['x'] = 0
                        self.maxPoint['y'] = 50000
                

                if(self.visible(allOtherPoints[i])):
                    res.append(allOtherPoints[i])

                if (allOtherPoints[i]['number'] != 0 and allOtherPoints[i]['number'] != 1):
                    for incidentVertex in  allOtherPoints[i]['incidentVertices']:

                        v1x = allOtherPoints[i]['point']['x'] - currentVertex['point']['x']
                        v1y = allOtherPoints[i]['point']['x'] - currentVertex['point']['y']

                        v2x = self.allPoints[incidentVertex]['point']['x'] - currentVertex['point']['x']
                        v2y = self.allPoints[incidentVertex]['point']['y'] - currentVertex['point']['y']

                        crossProduct = v1x*v2y - v2x*v1y
                        edge = Edge(allOtherPoints[i], self.allPoints[incidentVertex])

                        if (crossProduct < 0):
                            self.avlTree.delete(edge)
                            self.lastMin = self.avlTree.findMinimum()
                        elif crossProduct > 0:
                            self.avlTree.insert(edge)

        return res

                



    def visible(self, testVertex):
        if not self.prevVertex == None:
            slopePrev = (-(self.fixedPoint.y - self.prevVertex.y)) / (-(self.prevVertex.x - self.fixedPoint.x))
            slopeCurr = (-(self.fixedPoint.y - testVertex.y)) / (-(testVertex.x - self.fixedPoint.x))

            if (round(slopePrev * 1000) / 1000 == round(slopeCurr * 1000) / 1000):
                if not self.prevVertexVisible:
                    self.prevVertex = testVertex
                else:
                    if (testVertex.polygon == self.prevVertex.polygon):
                        if self.prevVertex.number not in testVertex.incidentVertices:
                            if self.visibleSamePolygon(testVertex, self.prevVertex):
                                self.prevVertex = testVertex
                                self.prevVertexVisible = False
                                return False





    def visibleSamePolygon(self, testingVertex, fixedVertex):
        if self.avlTree != None and (not self.undefined(self.allPoints.node)):
            leftmostEdge = self.avlTree.findMinimum()

            if leftmostEdge == None:
                fixedIncidentA = self.allPoints[fixedVertex.incidentVertices[0]]
                fixedIncidentB = self.allPoints[fixedVertex.incidentVertices[1]]


                v1x = fixedIncidentA.x - fixedVertex.x
                v1y = fixedVertex.y - fixedIncidentA.y

                v2x = fixedIncidentB.x - fixedVertex.x
                v2y = fixedVertex.y - fixedIncidentB.y

                v3x = testingVertex.x - fixedVertex.x
                v3y = fixedVertex.y - testingVertex.y


                crossProduct3 = v1x*v2y - v2x*v1y

                crossProduct1  = v1x*v3y - v3x*v1y 
                crossProduct2  = v2x*v3y - v3x*v2y 

                if (crossProduct1 >= 0 and crossProduct2 <= 0):
                    return False
                else:
                    if (crossProduct3 < 0):
                        if (crossProduct1 < 0 and crossProduct2 > 0):
                            return True
                        else:
                            return False

                    else:
                        return True

            
            pointA = self.lineIntersectionPoint(0, 0, self.maxPoint.x, self.maxPoint.y, leftmostEdge.vertexA.x - fixedVertex.x, fixedVertex.y - leftmostEdge.vertexA.y, leftmostEdge.vertexB.x - fixedVertex.x, fixedVertex.y - leftmostEdge.vertexB.y)
            distA = self.distance({'x':0, 'y':0}, pointA)
            roundedA = round(distA * 1000) / 1000


            distB  = self.distance({'x':0, 'y':0}, {'x':testingVertex.x - fixedVertex.x, 'y': fixedVertex.y - testingVertex.y})
            roundedB = round(distB * 1000) / 1000

            if (roundedA < roundedB):
                return False
            else:
                fixedIncidentA = self.allPoints[fixedVertex.incidentVertices[0]]
                fixedIncidentB = self.allPoints[fixedVertex.incidentVertices[1]]

                v1x = fixedIncidentA.x - fixedVertex.x
                v1y = fixedVertex.y - fixedIncidentA.y

                v2x = fixedIncidentB.x - fixedVertex.x
                v2y = fixedVertex.y - fixedIncidentB.y

                v3x = testingVertex.x - fixedVertex.x
                v3y = fixedVertex.y - testingVertex.y


                crossProduct3 = v1x*v2y - v2x*v1y

                crossProduct1  = v1x*v3y - v3x*v1y 
                crossProduct2  = v2x*v3y - v3x*v2y 

                if (crossProduct1 >= 0 and crossProduct2 <= 0):
                    return False
                else:
                    if (crossProduct3 < 0):
                        if (crossProduct1 < 0 and crossProduct2 > 0):
                            return True
                        else:
                            return False
                    else:
                        return True

        else:
            fixedIncidentA = self.allPoints[fixedVertex.incidentVertices[0]]
            fixedIncidentB = self.allPoints[fixedVertex.incidentVertices[1]]

            v1x = fixedIncidentA.x - fixedVertex.x
            v1y = fixedVertex.y - fixedIncidentA.y

            v2x = fixedIncidentB.x - fixedVertex.x
            v2y = fixedVertex.y - fixedIncidentB.y

            v3x = testingVertex.x - fixedVertex.x
            v3y = fixedVertex.y - testingVertex.y


            crossProduct3 = v1x*v2y - v2x*v1y

            crossProduct1  = v1x*v3y - v3x*v1y 
            crossProduct2  = v2x*v3y - v3x*v2y 


            if (crossProduct1 >= 0 and crossProduct2 <= 0):
                return False
            else:
                if (crossProduct3 < 0):
                    if (crossProduct1 < 0 and crossProduct2 > 0):
                        return True
                    else:
                        return False
                else:
                    return True










        

    def lineIntersection(self, x1, y1, x2, y2, x3, y3, x4, y4):

        try:
            uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
        except:
            if ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) > 0:
                uA = float('inf')
            else:
                uA = -float('inf')

        try:
            uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
        except:
            if ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) > 0:
                uB = float('inf')
            else:
                uB = -float('inf')

        if uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1:
            return True

        return False




    def distance(self, p1, p2): 
        return sqrt((p1["x"]-p2["x"])**2+(p1["y"]-p2["y"])**2)



    def dijkstra(self):
        startPoint = self.allPoints[0]
        distances = {}
        visitedVertices = {}
        previousVertices = {}

        queue = PriorityQueue()

        for point in self.allPoints:
            distances[point['number']] = INFINITY
            previousVertices[point['number']] = None

        distances[startPoint['number']] = 0

        queue.put(distances[startPoint['number']], startPoint['number'])

        while queue.not_empty:
            currentVertex = self.allPoints[queue.get()]
            neighbors = self.graph.AdjList[currentVertex['number']]
            for neighbor in neighbors:
                if visitedVertices[neighbors]:
                    pass

            print('not Empty')
            break


        return None








    def undefined(self, item):
        try:
            item
            return False
        except NameError:
            return True


    def compareEdges(self, a, b):
        if (a == None and b == None) or (self.undefined(a) and self.undefined(b)) or (a == None and self.undefined(b)) or (self.undefined(a) and b == None):
            return 0

        if a == None or self.undefined(a):
            return 1

        if b == None or self.undefined(b):
            return -1

        if (a.vertexA['point']['x'] == b.vertexA['point']['x'] and a.vertexA['point']['y'] == b.vertexA['point']['y'] and a.vertexB['point']['x'] == b.vertexB['point']['x'] and a.vertexB['point']['y'] == b.vertexB['point']['y']):
            return 0
        
        if (a.vertexA['point']['x'] == b.vertexB['point']['x'] and a.vertexA['point']['y'] == b.vertexB['point']['y'] and a.vertexB['point']['x'] == b.vertexA['point']['x'] and a.vertexB['point']['y'] == b.vertexA['point']['y']):
            return 0

        pointA = self.lineIntersectionPoint(0, 0, self.maxPoint['x'], self.maxPoint['y'], a.vertexA['point']['x'] - self.fixedPoint['point']['x'], self.fixedPoint['point']['y'] - a.vertexA['point']['y'], a.vertexB['point']['x'] - self.fixedPoint['point']['x'], self.fixedPoint['point']['y'] - a.vertexB['point']['y'])
        pointB = self.lineIntersectionPoint(0, 0, self.maxPoint['x'], self.maxPoint['y'], b.vertexA['point']['x'] - self.fixedPoint['point']['x'], self.fixedPoint['point']['y'] - b.vertexA['point']['y'], b.vertexB['point']['x'] - self.fixedPoint['point']['x'], self.fixedPoint['point']['y'] - b.vertexB['point']['y'])

        distA = self.distance({'x':0, 'y':0}, pointA)
        roundedA = round(distA * 1000) / 1000

        distB = self.distance({'x':0, 'y':0}, pointB)
        roundedB = round(distB * 1000) / 1000

        if (roundedA < roundedB):
            return -1
        elif (roundedA > roundedB):
            return 1
        else:
            point1 = a.vertexA
            point2 = a.vertexB
            point3 = b.vertexA
            point4 = b.vertexB


            if (point1['point']['x'] - self.fixedPoint['point']['x'] == round(pointA['x']) and self.fixedPoint['point']['y'] - point1['point']['y'] == round(pointA['y'])):
                if (point3['point']['x'] - self.fixedPoint['point']['x'] == round(pointA['x']) and self.fixedPoint['point']['y'] - point3['point']['y'] == round(pointA['y'])):
                    a1 = pow((point2['point']['x'] - self.fixedPoint['x']) - (point1['point']['x'] - self.fixedPoint['x']), 2) + pow((self.fixedPoint['y'] - point2['point']['y']) - (self.fixedPoint['y'] - point1['point']['y']), 2)
                    b1 = pow((point2['point']['x'] - self.fixedPoint['x']), 2) + pow((self.fixedPoint['y'] - point2['point']['y']), 2)
                    c1 = pow((point1['point']['x'] - self.fixedPoint['x']), 2) + pow((self.fixedPoint['y'] - point1['point']['y']), 2)
                    cosValueC = (a1 + c1 - b1) / (2 * sqrt(a1) * sqrt(c1))
                    angleC = math.acos(cosValueC)

                    a2 = pow((point4['point']['x'] - self.fixedPoint['x']) - (point1['point']['x'] - self.fixedPoint['x']), 2) + pow((self.fixedPoint['y'] - point4['point']['y']) - (self.fixedPoint['y'] - point1['point']['y']), 2)
                    b2 = pow((point4['point']['x'] - self.fixedPoint['x']), 2) + pow((self.fixedPoint['y'] - point4['point']['y']), 2)
                    c2 = pow((point1['point']['x'] - self.fixedPoint['x']), 2) + pow((self.fixedPoint['y'] - point1['point']['y']), 2)
                    cosValueD = (a2 + c2 - b2) / (2 * sqrt(a2) * sqrt(c2))
                    angleD = math.acos(cosValueD)

                    if (angleC < angleD):
                        return -1
                    else:
                        return 1

                else:
                    point2['point']['x']
                    a1 = pow((point2['point']['x'] - self.fixedPoint['point']['x']) - (point1['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point2['point']['y']) - (self.fixedPoint['point']['y'] - point1['point']['y']), 2)
                    b1 = pow((point2['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point2['point']['y']), 2)
                    c1 = pow((point1['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point1['point']['y']), 2)
                    cosValueC = (a1 + c1 - b1) / (2 * sqrt(a1) * sqrt(c1))
                    angleC = math.acos(cosValueC)

                    a2 = pow((point3['point']['x'] - self.fixedPoint['point']['x']) - (point1['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point3['point']['y']) - (self.fixedPoint['point']['y'] - point1['point']['y']), 2)
                    b2 = pow((point3['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point3['point']['y']), 2)
                    c2 = pow((point1['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point1['point']['y']), 2)
                    cosValueD = (a2 + c2 - b2) / (2 * sqrt(a2) * sqrt(c2))
                    angleD = math.acos(cosValueD)

                    if (angleC < angleD):
                        return -1
                    else:
                        return 1


            else:
                if (point3['point']['x'] - self.fixedPoint['point']['x'] == round(pointA['x']) and self.fixedPoint['point']['y'] - point3['point']['y'] == round(pointA['y'])):
                    a1 = pow((point1['point']['x'] - self.fixedPoint['point']['x']) - (point2['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point1['point']['y']) - (self.fixedPoint['point']['y'] - point2['point']['y']), 2)
                    b1 = pow((point1['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point1['point']['y']), 2)
                    c1 = pow((point2['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point2['point']['y']), 2)
                    cosValueC = (a1 + c1 - b1) / (2 * sqrt(a1) * sqrt(c1))
                    angleC = math.acos(cosValueC)

                    a2 = pow((point4['point']['x'] - self.fixedPoint['point']['x']) - (point2['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point4['point']['y']) - (self.fixedPoint['point']['y'] - point2['point']['y']), 2)
                    b2 = pow((point4['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point4['point']['y']), 2)
                    c2 = pow((point2['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point2['point']['y']), 2)
                    cosValueD = (a2 + c2 - b2) / (2 * sqrt(a2) * sqrt(c2))
                    angleD = math.acos(cosValueD)

                    if (angleC < angleD):
                        return -1
                    else:
                        return 1

                else:
                    a1 = pow((point1['point']['x'] - self.fixedPoint['point']['x']) - (point2['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point1['point']['y']) - (self.fixedPoint['point']['y'] - point2['point']['y']), 2)
                    b1 = pow((point1['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point1['point']['y']), 2)
                    c1 = pow((point2['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point2['point']['y']), 2)
                    
                    
                    try:
                        cosValueC = (a1 + c1 - b1) / (2 * sqrt(a1) * sqrt(c1))
                    except:
                        if (a1 + c1 - b1) > 0:
                            cosValueC = 1
                        else:
                            cosValueC = -1
                    
                    
                    angleC = math.acos(cosValueC)

                    a2 = pow((point3['point']['x'] - self.fixedPoint['point']['x']) - (point2['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point3['point']['y']) - (self.fixedPoint['point']['y'] - point2['point']['y']), 2)
                    b2 = pow((point3['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point3['point']['y']), 2)
                    c2 = pow((point2['point']['x'] - self.fixedPoint['point']['x']), 2) + pow((self.fixedPoint['point']['y'] - point2['point']['y']), 2)

                    try:
                        cosValueD = (a2 + c2 - b2) / (2 * sqrt(a2) * sqrt(c2))
                    except:
                        if (a2 + c2 - b2) > 0:
                            cosValueD = 1
                        else:
                            cosValueD = -1


                    angleD = math.acos(cosValueD)

                    if (angleC < angleD):
                        return -1
                    else:
                        return 1


                

                    
                    








    def lineIntersectionPoint(self, x1, y1, x2, y2, x3, y3, x4, y4):
        try:
            uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
        except:
            if ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) > 0:
                uA = float('inf')
            else:
                uA = -float('inf')

        try:
            uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
        except:
            if ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) > 0:
                uB = float('inf')
            else:
                uB = -float('inf')

        intersectionX = x1 + (uA * (x2 - x1))
        intersectionY = y1 + (uA * (y2 - y1))

        return {'x': intersectionX, 'y': intersectionY}
            


