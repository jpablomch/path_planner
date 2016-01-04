# path_planner

I developed this library as an alternative for simple path planning using an map image. We are using this library in an Assistive Indoor Navigation app at the City College Robotics Lab. 

# How to use it 

## Initilize a PathPlanner object 

We use a 2D byte array that represents the occupancy image. The path planner has a set of states that we can use when implementing the logic of a navigation app.

PathPlanner pathPlanner = new PathPlanner(yourByteArray); 
pathPlanner.pState = PathPlanner.PlannerState.NO_TASK_SELECTED;

## Path Planning 

### We first change the state of the path planner. 

mPlanner.pState = PathPlanner.PlannerState.GOING_TO_DESTINATION; // TODO: This should be set automatically. 

### Next, we set the origin and the destination, and request a path to the destination. 
pathPlanner.setOrigin(x1, y1);
pathPlanner.setDestination(x2, y2); 

### We request the creation of a path, and store the path.  
<boolean flag> = pathPlanner.findPath(); 
pathPlanner.storePath(); 

### Now we have a list of waypoints in storedPath. We can access a waypoint on the path using its position value. However, we have to consider that waypoints have been stored in reversed order.

Waypoint waypoint = pathPlanner.storedPath.get(indexOfWaypoint);

if you want to get and remove the waypoint, you might want to use:

Waypoint waypoint = pathPlanner.storedPath.remove(indexOfWaypoint);




