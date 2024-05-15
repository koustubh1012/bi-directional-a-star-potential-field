import cv2
import numpy as np
import heapq as hq

canvas = np.ones((300,600,3))*255   # creating a frame for video generation
obstacle_set = set()             # set to store the obstacle points
obstacle_list = []               # list to store the obstacle points in order for videp

c2c_node_grid = [[float('inf')] * 300 for _ in range(600)]       # create a 2D array for storing cost to come
tc_node_grid = [[float('inf')] * 300 for _ in range(600)]        # create a 2D array for storing cost to come
closed_set = set()               # set to store the value of visited and closed points
closed_list = []

T = 1

'''
Loop to define the obstacle points in the map
'''
for y in range(300):                                       # loop to define the obstacle points : x
    for x in range(600):                                  # loop to define the obstacle points : y
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
        if ((x-112)**2 + (y-242.5)**2 <= (40+T)**2):
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
        # Points in the Circle shaped obstacle
        elif ((x-263)**2 + (y-90)**2 <= (70+T)**2):
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1
        # Points in the Circle shaped obstacle
        elif ((x-445)**2 + (y-220)**2 <= (37.5+T)**2):
            obstacle_set.add((x,y))                        # add the points to the obstacle set
            obstacle_list.append((x,y))                    # add the points to the obstacle list
            c2c_node_grid[x][y] = -1                       # mark the points in the cost to come grid with -1
            tc_node_grid[x][y] = -1                        # mark the points in the total cost grid with -1


# Mark the obstacle points in the frame, including points after bloating
for point in obstacle_list:                            # loop to mark the obstacle points
    canvas[point[1],point[0]] = [255, 0, 0]            # mark the obstacle points with blue color

START_X = 0.0
START_Y = 150.0
GOAL_X = 600.0
GOAL_Y = 150.0

start = np.array([START_X, START_Y])
goal = np.array([GOAL_X, GOAL_Y])

Q_ROBOT = -1                     # Charge on the robot
Q_OBSTACLE = -1                  # Charge on the obstacle
Q_GOAL = 100                     # Charge on the goal

current_node = start

W1 = 7000                        # Weight/constant for the attractive force
W2 = 1                           # Weight/constant for the repulsive force

goal_reached = False

# Loop to execute the algorithm
while(not goal_reached):
    f_obs = np.array([0.0, 0.0])
    x = round(current_node[0])
    y = round(current_node[1])
    cv2.circle(canvas, (x,y), 3, (0, 0 ,255), -1)

    goal_vector = current_node - goal          # Goal vector
    d1 = np.linalg.norm(goal_vector)           # distance to goal

    f_mag_goal = W1 * Q_ROBOT * Q_GOAL/(d1**2)      # Attractive force towards the goal
    f_goal = f_mag_goal * (goal_vector/d1)          # Attractive Force vector to the goal 

    if d1<=5:                                   # Check if the goal is reached
        print("GOAL REACHED")
        goal_reached = True
        break
    
    # Loop to calculate the repulsive force
    for point in obstacle_list:
        obs_point = np.array([point[0], point[1]])           # Obstacle point
        obs_vector = current_node - obs_point                # vector to obstacle point
        d2 = np.linalg.norm(obs_vector)                      # distance to obstacle point
        f_mag_obs = W2*Q_ROBOT*Q_OBSTACLE/(d2**2)            # repulsive force magnitude 
        f_obs += f_mag_obs * (obs_vector/d2)                 # repulsive force vector

    f = f_goal + f_obs                                       # Resultant force vector

    current_node += (f*10/np.linalg.norm(f))                 # Move the node towards the direction of the force

canvas_flipped = cv2.flip(canvas,0)
cv2.imshow("Potential Field Path Planning", canvas_flipped)
cv2.waitKey(0)
cv2.destroyAllWindows()