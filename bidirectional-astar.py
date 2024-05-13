# import libraries

import cv2
import numpy as np
import heapq as hq
import math
import time

canvas = np.ones((300,600,3))   # creating a frame for video generation
obstacle_set = set()             # set to store the obstacle points
obstacle_list = []               # list to store the obstacle points in order for videp

c2c_node_grid = [[float('inf')] * 300 for _ in range(600)]       # create a 2D array for storing cost to come
tc_node_grid = [[float('inf')] * 300 for _ in range(600)]        # create a 2D array for storing cost to come
closed_set = set()               # set to store the value of visited and closed points
closed_list = []
visited={}
visited_node_list = []

closed_set_2 = set()               # set to store the value of visited and closed points
closed_list_2 = []
visited_2={}
visited_node_list_2 = []

# C = int(input("Enter the clearance from the obstacle in mm: "))     # Get clearance from the user
# C = C/10
C = 0

# All units are in cm
R = 66/20                                                  # Robot wheel radius
r = 22                                                   # Robot radius
L = 28.7                                                   # Robot wheel track
T = 1                                                 # Total clearance

t_max = 3.5
t_min = 0.1

x_goal = 0  # Initialize the goal x coordinate
y_goal = 0  # Initialize the goal y coordinate
x_start = 0 # Initialize the start x coordinate
y_start = 0 # Initialize the start y coordinate
node=(0, 0, 1, [], (x_start, y_start), 0)

'''
Loop to define the obstacle points in the map
'''
for y in range(300):                                       # loop to define the obstacle points : x
    for x in range(600):                                  # loop to define the obstacle points : y
        canvas[y,x] = [255,255,255]                        # mark the points in the frame with white color
        if (0<=y<=T):                                      # points in the bottom boundary
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
        elif (300-T<=y<300):                               # points in the top boundary
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
        # Points in the Circle shaped obstacle
        elif ((x-112)**2 + (y-242.5)**2 <= (40)**2):
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
        # Points in the Circle shaped obstacle
        elif ((x-263)**2 + (y-90)**2 <= (70)**2):
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
        # Points in the Circle shaped obstacle
        elif ((x-445)**2 + (y-220)**2 <= (37.5)**2):
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1

# Mark the obstacle points in the frame, including points after bloating
for point in obstacle_list:                            # loop to mark the obstacle points
    canvas[point[1],point[0]] = [255, 0, 0]            # mark the obstacle points with blue color

x_start =0
y_start = 150
theta_start = 0
x_goal = 600
y_goal = 150
rpm1 = 5
rpm2 = 10
goal_found = False
theta_goal = 180
reached_from_start = False
reached_from_goal = False

initial_node = (0, 0, 1, [], (x_start, y_start), theta_start)
initial_node_2 = (0, 0, 1, [], (x_goal, y_goal), theta_goal)

min_rpm = min(rpm1,rpm2)

t = round(((t_max - t_min)*(min_rpm - 75)/(5 - 75)) + t_min, 2)                     # Calculate time step
print("Calculated time step: ", t)

# Funtion to update the visted nodes
def visited_node(node):
    visited.update({node[2]:node[4]})
    
def visited_node_2(node_2):
    visited_2.update({node_2[2]:node_2[4]})

def action1(node,rpm1,rpm2):
    ul = 2*math.pi*rpm1/60
    ur = 2*math.pi*rpm2/60
    new_heading = (node[5] + np.rad2deg(((R/L)*(ul - ur)*t))) % 360        # get the current heading of the robot
    x_vel = (R/2)*(ur+ul)*np.cos(np.deg2rad(new_heading))
    y_vel = (R/2)*(ur+ul)*np.sin(np.deg2rad(new_heading))
    x = node[4][0] + x_vel*t # calculate the new x coordinate
    y = node[4][1] + y_vel*t # calculate the new y coordinate
    x = round(x)
    y = round(y)
    c2c = node[1] + math.sqrt((x_vel*t)**2 + (y_vel*t)**2)                                    # calculate the cost to come
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)   # calculate the cost to goal
    tc = c2c + c2g                                   # calculate the total cost
    return (x,y),new_heading,tc,c2c                  # return the new node's coordinates, heading, total cost and cost to come

def action2(node,rpm1,rpm2):
    ul = 2*math.pi*rpm1/60
    ur = 2*math.pi*rpm2/60
    new_heading = (node[5] + np.rad2deg(((R/L)*(ul - ur)*t))) % 360        # get the current heading of the robot
    x_vel = (R/2)*(ur+ul)*np.cos(np.deg2rad(new_heading))
    y_vel = (R/2)*(ur+ul)*np.sin(np.deg2rad(new_heading))
    x = node[4][0] + x_vel*t # calculate the new x coordinate
    y = node[4][1] + y_vel*t # calculate the new y coordinate
    x = round(x)
    y = round(y)
    c2c = node[1] + math.sqrt((x_vel*t)**2 + (y_vel*t)**2)                                    # calculate the cost to come
    c2g = math.sqrt((y_start-y)**2 + (x_start-x)**2)   # calculate the cost to goal
    tc = c2c + c2g                                   # calculate the total cost
    return (x,y),new_heading,tc,c2c                  # return the new node's coordinates, heading, total cost and cost to come

action_lists=[(0,rpm1),(rpm1,0),(rpm1,rpm1),(rpm1,rpm2),(rpm2,rpm1),(0,rpm2),(rpm2,0),(rpm2,rpm2)]

start_time = time.time()
new_index = 1
open_list = []
hq.heappush(open_list,initial_node)        # Push initial node to the list
hq.heapify(open_list)                      # covers list to heapq data type

new_index_2 = 1
open_list_2 = []
hq.heappush(open_list_2,initial_node_2)        # Push initial node to the list
hq.heapify(open_list_2)                      # covers list to heapq data type

while(not goal_found):
    node = hq.heappop(open_list)       # pop the node with lowest cost to come
    closed_list.append(node[4])            # add the node coordinates to closed set
    closed_set.add(node[4])
    visited_node(node)                 # add the node to the visited list
    visited_node_list.append(node)
    index = node[2]                    # store the index of the current node
    parent_index = node[3]             # store the parent index list of current node
    
    node_2 = hq.heappop(open_list_2)       # pop the node with lowest cost to come
    closed_list.append(node_2[4])            # add the node coordinates to closed set
    closed_set_2.add(node_2[4])
    visited_node_2(node_2)                 # add the node to the visited list
    visited_node_list_2.append(node_2)
    index_2 = node_2[2]                    # store the index of the current node
    parent_index_2 = node_2[3]             # store the parent index list of current node

    # node_dist = math.sqrt((node[4][0]-x_goal)**2 + (node[4][1]-y_goal)**2)     # calculate the distance between the current node and goal node
    # if node_dist < 5:    # if the node is goal position, exit the loop
    #     print("Goal reached")
    #     break
    if node[4] in closed_set_2:
        print("Goal reached from start")
        path_1 = node
        # print(path_1[4])
        reached_from_start = True
        break
    if node_2[4] in closed_set:
        print("Goal reached from goal")
        path_2 = node_2
        # print(path_2[4])
        reached_from_goal = True
        break
    
    for action_set in action_lists:
        point, new_heading, tc, c2c = action1(node,action_set[0],action_set[1])
        if point not in obstacle_set and point not in closed_set and 0<=point[0]<600 and 0<=point[1]<300:           # check if the new node is in the obstacle set or visited list
            x = int(point[0])                                                    # get the x coordinathe te of the new node
            y = int(point[1])                                                    # get the y coordinate of the new node
            try:
                if tc<tc_node_grid[x][y]:                                       # check if the new cost to come is less than original cost to come
                    new_parent_index = parent_index.copy()                      # copy the parent index list of the current node
                    new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
                    new_index+=1                                                # increment the index
                    tc_node_grid[x][y] = tc                                     # Update the new total cost
                    c2c_node_grid[x][y] = c2c                                   # Update the new cost to come
                    new_node = (tc, c2c, new_index, new_parent_index, (x,y), new_heading) # create the new node
                    hq.heappush(open_list, new_node)                            # push the new node to the open list
            except:
                print(x,y)
                
    for action_set in action_lists:
        point, new_heading, tc, c2c = action2(node_2,action_set[0],action_set[1])
        if point not in obstacle_set and point not in closed_set_2 and 0<=point[0]<600 and 0<=point[1]<300:           # check if the new node is in the obstacle set or visited list
            x = int(point[0])                                                    # get the x coordinathe te of the new node
            y = int(point[1])                                                    # get the y coordinate of the new node
            try:
                if tc<tc_node_grid[x][y]:                                       # check if the new cost to come is less than original cost to come
                    new_parent_index_2 = parent_index_2.copy()                      # copy the parent index list of the current node
                    new_parent_index_2.append(index_2)                              # Append the current node's index to the new node's parent index list
                    new_index_2+=1                                                # increment the index
                    tc_node_grid[x][y] = tc                                     # Update the new total cost
                    c2c_node_grid[x][y] = c2c                                   # Update the new cost to come
                    new_node_2 = (tc, c2c, new_index_2, new_parent_index_2, (x,y), new_heading) # create the new node
                    hq.heappush(open_list_2, new_node_2)                            # push the new node to the open list
            except:
                print(x,y)



print("Actual goal reached :",(node[4][0])*10, (node[4][1]-150)*10)


# path = node[3]            # Get the parent node list
# path_2 = node_2[3]
# print(path)
# print(path_2)
counter = 0               # counter to count the frames to write on video

fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4 format
video_writer = cv2.VideoWriter('output/new_output.mp4', fourcc, 60, (600, 300)) # Video writer object

'''
Loop to mark the explored nodes in order on the frame
'''
print("Exploring map")


# print(closed_set)
for node in closed_list:                                                  # loop to mark the explored nodes
    # canvas[node[1], node[0]] = [0, 255, 0]                               # mark the explored nodes with green color
    cv2.circle(canvas, node, 1, (0,255,0), -1)             # mark the goal point with red color
    counter +=1                                                          # increment the counter
    if counter%25 == 0 or counter == 0:                                 # check if the counter is divisible by 500
        cv2.circle(canvas,(x_start, y_start), 5, (0,0,255), -1)             # mark the goal point with red color
        cv2.circle(canvas,(x_goal, y_goal), 5, (255,0,255), -1)             # mark the goal point with red color
        canvas_flipped = cv2.flip(canvas,0)                              # flip the frame
        canvas_flipped_uint8 = cv2.convertScaleAbs(canvas_flipped)       # convert the frame to uint8
        video_writer.write(canvas_flipped_uint8)                         # write the frame to video
        
        
if reached_from_start:
    '''
    Loop to mark the path created
    '''
    print("Backtracking")

    for index in path_1[3]:                                                        # loop to mark the path
        coord=visited[index]                                                  # get the coordinates of the node
        cv2.circle(canvas, (coord[0],coord[1]), 1, [0,0,0], -1)               # mark the path with black color
        canvas_flipped = cv2.flip(canvas, 0)
        canvas_flipped_uint8 = cv2.convertScaleAbs(canvas_flipped)            # convert the frame to uint8
    video_writer.write(canvas_flipped_uint8)                              # write the frame to video

elif reached_from_goal:
    '''
    Loop to mark the path created
    '''
    print("Backtracking 2")

    for index in path_2[3]:                                                        # loop to mark the path
        coord=visited_2[index]                                                  # get the coordinates of the node
        cv2.circle(canvas, (coord[0],coord[1]), 1, [0,0,0], -1)               # mark the path with black color
        canvas_flipped = cv2.flip(canvas, 0)
        canvas_flipped_uint8 = cv2.convertScaleAbs(canvas_flipped)            # convert the frame to uint8
        video_writer.write(canvas_flipped_uint8)                              # write the frame to video
        
    # for node in visited_node:
    #     if node[4] == path_2[4]:
    #         print(node)
    #         print(node[4])
    #         print(path_2[4])


'''
Loop to add some additional frames at the end of the video
'''
for i in range(150):
    video_writer.write(canvas_flipped_uint8)                               # write the frame to video

print("Video Processed")                                                   # print message

video_writer.release()                                                     # release the video writer

end_time = time.time()                                                     # get the end time of the program
print(f"The runtime of my program is {end_time - start_time} seconds.")    # print the runtime of the program

cv2.imshow("Path",canvas_flipped_uint8)
cv2.waitKey(0)
cv2.destroyAllWindows()