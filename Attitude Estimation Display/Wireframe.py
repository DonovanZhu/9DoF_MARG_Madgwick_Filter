import numpy as np
import math
from scipy.spatial.transform import Rotation as R


# Node stores each point of the block
class Node:
    def __init__(self, coordinates, color):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.z = coordinates[2]
        self.color = color


# Face stores 4 nodes that make up a face of the block
class Face:
    def __init__(self, nodes, color):
        self.nodeIndexes = nodes
        self.color = color


# Wireframe stores the details of a block
class Wireframe:
    def __init__(self):
        self.nodes = []
        self.edges = []
        self.faces = []
        self.ypr = []

    def addNodes(self, nodeList, colorList):
        for node, color in zip(nodeList, colorList):
            self.nodes.append(Node(node, color))

    def addFaces(self, faceList, colorList):
        for indexes, color in zip(faceList, colorList):
            self.faces.append(Face(indexes, color))

    def quatRotate(self, YPR):
        self.ypr = YPR

    def rotatePoint(self, point):

        roll = -self.ypr[0]
        pitch = self.ypr[1]
        yaw = -self.ypr[2]
        yawMatrix = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                              [math.sin(yaw), math.cos(yaw), 0],
                              [0, 0, 1]])

        pitchMatrix = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                                [0, 1, 0],
                                [-math.sin(pitch), 0, math.cos(pitch)]])

        rollMatrix = np.array([[1, 0, 0],
                               [0, math.cos(roll), -math.sin(roll)],
                               [0, math.sin(roll), math.cos(roll)]])

        rotationMat = np.matmul(yawMatrix, np.matmul(pitchMatrix, rollMatrix))
        return np.matmul(rotationMat, point)

    def convertToComputerFrame(self, point):
        computerFrameChangeMatrix = np.array([[-1, 0, 0], [0, 0, -1], [0, -1, 0]])
        return np.matmul(computerFrameChangeMatrix, point)

    def outputNodes(self):
        print("\n --- Nodes --- ")
        for i, node in enumerate(self.nodes):
            print(" %d: (%.2f, %.2f, %.2f) \t Color: (%d, %d, %d)" %
                  (i, node.x, node.y, node.z, node.color[0], node.color[1], node.color[2]))

    def outputFaces(self):
        print("\n --- Faces --- ")
        for i, face in enumerate(self.faces):
            print("Face %d:" % i)
            print("Color: (%d, %d, %d)" % (face.color[0], face.color[1], face.color[2]))
            for nodeIndex in face.nodeIndexes:
                print("\tNode %d" % nodeIndex)
