# path_planner

I developed this library as an alternative for simple path planning using an map image. We are using this application in an Assistive Indoor Navigation app at the City College Robotics Lab. 

# How to use it 

## Initilize a PathPlanner object 

We use a 2D byte array that represents the occupancy image. The path planner has a set of states that we can use when implementing the logic of a navigation app. 

PathPlanner pathPlanner = new PathPlanner(yourByteArray); 
pathPlanner.pState = PathPlanner.PlannerState.NO_TASK_SELECTED;

## Create a path 

We change the state of the path planner. Next, we set the origin and the destination, and request a path to the destination. 

mPlanner.pState = PathPlanner.PlannerState.GOING_TO_DESTINATION;
pathPlanner.setOrigin(x1, y1);
pathPlanner.setDestination(x2, y2); 

### First, we need to set the origin. 

### Second, we need to set the destination. 

### Get list of waypoints

### Remove visited waypoints 



