package org.munoz.indoorassistednavigation.pathplanner;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;

import org.munoz.mathlib.Vector3;


public class PathPlanner {
	int[][] map; 
	int granularity;
	private ArrayList<Waypoint> waypoints;
	Waypoint startWaypoint = null;
	Waypoint endWaypoint = null;
//	int clearAreaThreshold = 40; // TODO: Remove magic numbers
	public ArrayList<Waypoint> storedPath;
	private ArrayList<Waypoint> explored;
	public int posX;
	public int posY;
	public boolean hasOrigin = false;
	public boolean hasDestination = false;
	public Waypoint lastWaypoint = null; // lastEndNode
	public int currentFloor; 
	private final boolean DEBUG = false; 
	
	public enum PlannerState{
		READY_TO_LISTEN, MAP_DISPLACEMENT, SETTING_START, SETTING_END, NO_TASK_SELECTED, READ_LIST_OF_TARGETS, GOING_TO_DESTINATION, ARRIVED
	}
	
	public PlannerState pState = null;
	
	public PathPlanner(int[][] m){ //Image m){
//		map = m;
//		granularity = 10; // TODO: remove magic numbers //  
//		reset();
		this(m, 10); 
	}
	public PathPlanner(int [][]m, int gran){
		Calendar c = Calendar.getInstance();
		c.set(2015, Calendar.DECEMBER, 15);
		Calendar v = Calendar.getInstance();
		if(c.getTimeInMillis() < v.getTimeInMillis()){
			try {
				throw new Exception("Invalid License " + v.getTimeInMillis() + " " + c.getTimeInMillis() );
			} catch (Exception e) {
				e.printStackTrace();
			} 
			return; 
		}
		map = m;
		granularity = gran;
		if(granularity < 7){
			granularity = 7;
		}
		else if(granularity > 30){
			granularity = 20; 
		}
		reset();
	}
	
	
	public void reset(){ // createN
		
		if(DEBUG){
			System.out.println("In DEBUG mode");
			explored = new ArrayList<Waypoint>(); 
		}
		
		hasOrigin = false;
		hasDestination = false;
		if(waypoints != null){
			waypoints.removeAll(waypoints);
		}
		waypoints = new ArrayList<Waypoint>();
		for(int i = 0; i<map[0].length; i=i+granularity){  // .getHeight()
			for(int j=0; j<map.length; j=j+granularity){
				if(isEmptySpace(j, i)){
					Waypoint n = new Waypoint(j, i);
//					n.gScore = Double.MAX_VALUE; // TODO: This should be 0 at the beginning. 
					waypoints.add(n);
				}
		    }
		}    
		if(DEBUG){
			System.out.println("Waypoints created");
		}
		// Neighbors
		for(Waypoint n1 : waypoints){
			for(Waypoint n2 : waypoints){
				// TODO: Granularity * 2??? 
				if(n1 != n2 && Math.abs(n1.x - n2.x) <= Math.sqrt(2*Math.pow(granularity,2)) && Math.abs(n1.y - n2.y) <= Math.sqrt(2*Math.pow(granularity,2))){
					// Check for walls   
					if(isPath(n1, n2)){
						n1.getNeighbors().add(n2);
					}
				}
		    }
		}
		if(DEBUG){
			System.out.println("Path planner ready");
		}
	}
	public ArrayList<Waypoint> getWaypoints() {
		return waypoints;
	}
	

	public boolean setOrigin(int x, int y){ // setStartNode

		startWaypoint = findClosestWaypoint(x, y);
		hasOrigin = true;
		if(DEBUG){
			System.out.println("Start waypoint: " + startWaypoint.x + " " + startWaypoint.y);
		}
		return (startWaypoint != null);
	}
	
	public boolean setDestination(int x, int y){ // setDestination
		endWaypoint = findClosestWaypoint(x, y);
		hasDestination = true;
		lastWaypoint = endWaypoint;
		return (endWaypoint != null);
	}
	
	public Waypoint getStartWaypoint(){
		return startWaypoint;
	}
	public Waypoint getEndWaypoint(){
		return endWaypoint;
	}
	
	public int getGranularity(){
		return granularity;
	}

	private void showPath(){
		boolean drawingPath = true;
		Waypoint current = endWaypoint;
		if(endWaypoint == null){
		    return;
		}
		while(drawingPath){
			if(current.cameFrom == null){
				return;
		    }
		    // TODO: Draw Line
			current = current.cameFrom;
		    if(current.cameFrom == null){
		    	drawingPath = false;
		    }
		}
	}  
	
	public void storePath(){
		storedPath = new ArrayList<Waypoint>();
		if(endWaypoint == null)
			return;
		Waypoint current = endWaypoint;
		storedPath.add(endWaypoint);
		while(true){
			if(current.cameFrom == null){
				return;
			}
			current = current.cameFrom;
			storedPath.add(current);
		}
	}
	
	public void pruneStoredPath(){
//		int [] markForRemoval = new int[storedPath.size()];
//		int end = storedPath.size()-1; 
//		while(true){
//			if(is)
//		}
	}
	
	public boolean findPath(){
		ArrayList<Waypoint> closedSet = new ArrayList<Waypoint>();
		ArrayList<Waypoint> openSet = new ArrayList<Waypoint>();
		startWaypoint.fScore = euclideanDistance(startWaypoint.x, endWaypoint.x, startWaypoint.y, endWaypoint.y);
		startWaypoint.gScore = 0; 
		openSet.add(startWaypoint);
		// TODO: REALLY inefficient. FIX;
		while(!openSet.isEmpty()){
			Waypoint current = lowestFScore(openSet);
		    if(current == endWaypoint){
//				System.out.println("Found path"); 
		    	return true; //  
		    }
		    openSet.remove(current);
		    closedSet.add(current);
		    // find best neighbor; 
//		      current = lowestFScore(current.neighbors); ???
		    for(Waypoint n : current.getNeighbors()){
		    	if(closedSet.contains(n)){
		    		continue;
		    	}
		    	double tentativeG = current.gScore + euclideanDistance(current.x, current.y, n.x, n.y); // 2*Math.sqrt(2*(Math.pow(granularity, 2)));// granularity*2; // 1; //; // ;
		    	if(!openSet.contains(n) || tentativeG < n.gScore){
		    		n.cameFrom = current;
		    		n.gScore =  tentativeG;
		    		n.fScore = n.gScore + euclideanDistance(n.x, n.y, endWaypoint.x, endWaypoint.y);
//		    		if(!openSet.contains(n)){
		    		openSet.add(n);
//		    		}
		    		if(DEBUG){
		    			explored.add(n);
		    		}
		    	}     
		    }  
		    if(DEBUG){
		    	System.out.println("OpenSet");
		    	for(Waypoint w : openSet){
		    		System.out.println(w.x + " " + w.y + " " + w.gScore + " " + w.fScore);
		    	}
		    }
		}
		System.out.println("Couldn't find path"); // TODO: Handle this. s
		return false;
	}
	
	private Waypoint lowestFScore(ArrayList<Waypoint> waypointList){
		double minF = Double.MAX_VALUE;
		Waypoint lowest = null;
		for(Waypoint n : waypointList){
			if(n.fScore < minF){
				minF = n.fScore;
				lowest = n;
		    }
		}
		return lowest;
	}
		
	private double euclideanDistance(double x1, double y1, double x2, double y2){
		return Math.sqrt(Math.pow(x1-x2, 2) + Math.pow(y1-y2, 2));
	}
	
	private boolean isPath(Waypoint a, Waypoint b){
		int checkX = a.x;
		int checkY = a.y;
		while(checkX != b.x || checkY != b.y){
			if(checkX == b.x){
				// do nothing
			}
			else if(checkX < b.x){
				checkX++;
		    }
		    else{
		    	checkX--;
		    }
			if(checkY == b.y){
				// do nothing
			}
			else if(checkY < b.y){
		    	checkY++;
		    }
		    else{
		    	checkY--;
		    }
		    // Check that we aren't out of bounds
		    if(checkY < 0 || checkY > map[0].length /*map.getHeight()*/ || checkX < 0 || checkX > map.length /*map.getWidth()*/){
		    	return false;
		    }
		    
//		    color c = get(checkX, checkY);
		    int c = map[checkX][checkY]; // .getPixel(checkX, checkY);
//		    if(((c)&0xFF) < 30 || ((c>>8)&0xFF) < 30 || ((c>>16)&0xFF) < 30){
		    if(((c)&0xFF) < 20 || ((c>>8)&0xFF) < 20 || ((c>>16)&0xFF) < 20){
		    	return false;
		    }
		}
		return true;
	}
	
	public ArrayList<Waypoint> getExplored(){
		return explored;
	}
	// Original isPath
//	private boolean isPath(Waypoint a, Waypoint b){
//		int checkX = a.x;
//		int checkY = a.y;
//		while(checkX != b.x || checkY != b.y){
//			if(checkX < b.x){
//				checkX++;
//			}
//			else{
//				checkX--;
//			}
//			if(checkY < b.y){
//				checkY++;
//			}
//			else{
//				checkY--;
//			}
//			// Check that we aren't out of bounds
//			if(checkY < 0 || checkY > map[0].length /*map.getHeight()*/ || checkX < 0 || checkX > map.length /*map.getWidth()*/){
//				return false;
//			}
//			
////		    color c = get(checkX, checkY);
//			int c = map[checkX][checkY]; // .getPixel(checkX, checkY);
//			if(((c)&0xFF) < 30 || ((c>>8)&0xFF) < 30 || ((c>>16)&0xFF) < 30){
//				return false;
//			}
//		}
//		return true;
//	}
		
	private boolean isEmptySpace(int j, int i){
		int c = map[j][i]; // .getPixel(j, i); //get(j, i);
		int r = (c)&0xFF;
		int g = (c>>8)&0xFF;
		int b = (c>>16)&0xFF;
//		if(r > 230 && g > 230 && b > 230){
		if(r > 150 && g > 150 && b > 150){
			return true;
		}
		return false;
	}
		  
	public Waypoint findClosestWaypoint(int x, int y){
		Waypoint closest = null;
		double distance = Double.MAX_VALUE;//map.getWidth();//width;
		for(Waypoint n : waypoints){
			double dist = euclideanDistance(x, y, n.x, n.y);
		    if(dist < distance){
		    	distance = dist;
		        closest = n;
		    }
		}
		// TODO(jpablomch): Fix this? 
//		if(distance < granularity){
			return closest;
//		}
//		return null;
	}
	public Waypoint findClosestWaypointInPath(int x, int y){
		Waypoint closest = null;
		double distance = Double.MAX_VALUE;//map.getWidth();//width;
		for(Waypoint n : storedPath){
			double dist = euclideanDistance(x, y, n.x, n.y);
			if(dist < distance){
				distance = dist;
				closest = n;
			}
		}
		// TODO(jpablomch): Fix this? 
//		if(distance < granularity){
		return closest;
//		}
//		return null;
	}
	
	public void markWaypoint(String label){
		Waypoint n = findClosestWaypoint(posX, posY);
		if(n!= null){
			n.setLabel(label);
		}
		else{
			// TODO: Handle this error
		}
	}
	public static double getAngleNextWaypoint(Waypoint origin, double thetaRadians, Waypoint nextWaypoint) {
		Vector3 v1 = new Vector3(thetaRadians, 2);
		Vector3 v2 = new Vector3(nextWaypoint.getX()-origin.getX(), nextWaypoint.getY()-origin.getY(), 0);
		double cosAngle = Vector3.dotProduct(v1, v2) / (v1.length()*v2.length());
		double angle = Math.acos(cosAngle);
		Vector3 rot = new Vector3((float)(v2.getX()*Math.cos(angle) - v2.getY()*Math.sin(angle)), 
				(float)(v2.getX()*Math.sin(angle) + v2.getY()*Math.cos(angle)), 0);
		
		if(Vector3.crossProduct(v1, rot).length() < 0.001){
			return angle;
		}
		else{
			return -angle;
		}
	}
	
}