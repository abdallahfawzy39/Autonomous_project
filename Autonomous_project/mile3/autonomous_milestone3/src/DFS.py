#!/usr/bin/env python

import numpy as np #Imports Numpy package full of functions and methods that can be used.
import rospy
import cv2 
from std_msgs.msg import Float32MultiArray

def DFS(Map,Start,Goal,Direction):  # Direction for CCW -1  for CW 1

    Flag = False # Flag indicating if goal node is found
    
    # Initializing a stack 
    stack = []
    stack.append(Start)

    # Visited Nodes
    Visited = []

    # Parent Node Mapping
    Parents = np.zeros([10,10,2],dtype = int)


    while stack:   # If stack is not empty 

        Node = stack.pop()  # Pop last element in the stack

        if Node==Goal:   # Did we reach the goal?
            Flag = True
            break

        Neighbours = get_Neighbours(Node,Direction) # Get neighbours of this node

        for N in Neighbours: # For each neighbour of this node, in reverse order
            if not N in Visited and Map[N[0],N[1]]!=0:   # if this neighbour wasnt visited before and doesnt equal 0 (ie not an obstacle)
                stack.append(N)      # add to the stack
                Visited.append(N)     # add it to the visited
                Parents[N[0],N[1]] = Node  # add its parent node 

    if Flag == False:
        print('No Path Found')

    if Flag == True:
        path = get_Path(Parents,Goal,Start) # this function returns the path to the goal 
        return path


    

def get_Path(Parents,Goal,Start):

    path = []
    Node = Goal  # start backward 
    path.append(Goal)
   
    while not np.array_equal(Node, Start):

       path.append(Parents[Node[0],Node[1]])  # add the parent node 
       Node = Parents[Node[0],Node[1]]        # search for the parent of that node 
     
    path.reverse()
    return path

    

def get_Neighbours(Node,Direction):    # Returns neighbours of the parent node   Direction = 1 for Clockwise/ Direction = -1 for CCW 

    Neighbours = []
    #starting from East
    if Direction==1:
        Neighbours.append([Node[0],Node[1]+1])
        Neighbours.append([Node[0]+1,Node[1]+1])
        Neighbours.append([Node[0]+1,Node[1]])
        Neighbours.append([Node[0]+1,Node[1]-1])
        Neighbours.append([Node[0],Node[1]-1])
        Neighbours.append([Node[0]-1,Node[1]-1])
        Neighbours.append([Node[0]-1,Node[1]])
        Neighbours.append([Node[0]-1,Node[1]+1])
                        
    if Direction==-1:
        Neighbours.append([Node[0],Node[1]+1])
        Neighbours.append([Node[0]-1,Node[1]+1])
        Neighbours.append([Node[0]-1,Node[1]])
        Neighbours.append([Node[0]-1,Node[1]-1])
        Neighbours.append([Node[0],Node[1]-1])
        Neighbours.append([Node[0]+1,Node[1]-1])
        Neighbours.append([Node[0]+1,Node[1]])
        Neighbours.append([Node[0]+1,Node[1]+1])

    return Neighbours

if __name__ == '__main__':     # Main function that is executed 

    img = cv2.imread('/home/dell/catkin_ws/src/autonomous_milestone3/map.png')  #Read image 
    grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #Convert RGB image to grayscale
    ret, bw_img = cv2.threshold(grayImage,0,1,cv2.THRESH_BINARY) #Convert grayscale image to binary
    print(bw_img)
    bw_img = bw_img.astype(np.uint8)  
    path = DFS(bw_img,[1,1],[6,7],1) 
    print("DFS METHOD")
    print(path)
    
    
    rospy.init_node('DFS_path_publisher', anonymous=True)
    path_pub = rospy.Publisher('/path_points', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(1)

    for point in path:
        point_msg = Float32MultiArray()
        point_msg.data = point
        path_pub.publish(point_msg)
        rate.sleep()
    




