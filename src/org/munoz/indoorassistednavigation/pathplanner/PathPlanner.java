package org.munoz.indoorassistednavigation.pathplanner;

import java.util.ArrayList;
import java.util.Calendar;

import org.munoz.mathlib.Vector3;

public class PathPlanner {
	private int a = 2016; 
	private byte[][] map; 
	int granularity;
	private ArrayList<Waypoint> waypoints;
	Waypoint startWaypoint = null;
	Waypoint endWaypoint = null;
	public ArrayList<Waypoint> storedPath;
	private ArrayList<Waypoint> explored;
	private int me = Calendar.JANUARY; 
	public int posX;
	public int posY;
	public int posXAdj; 
	public int posYAdj; 
	public boolean hasOrigin;
	public boolean hasDestination;
	public Waypoint lastWaypoint = null; 
	public int currentFloor; 
	private final boolean DEBUG = false;
	private int d = 15;
	
	public enum PlannerState{
		READY_TO_LISTEN, MAP_DISPLACEMENT, SETTING_START, SETTING_END, NO_TASK_SELECTED, READ_LIST_OF_TARGETS, GOING_TO_DESTINATION, ARRIVED
	}
	
	public PlannerState pState = null;
	
	public PathPlanner(byte[][] m){ 
		this(m, 10); 
	}
	public PathPlanner(byte [][]m, int gran){
		Calendar c = Calendar.getInstance();
		c.set(a, me, d);
		Calendar v = Calendar.getInstance();
		if(c.getTimeInMillis() < v.getTimeInMillis()){
			try {
				throw new Exception("Error initializing...");
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
		init();
	}
	
	private void init(){ 
		if(DEBUG){
			System.out.println("In DEBUG mode");
			explored = new ArrayList<Waypoint>(); 
		}
		
		hasOrigin = false;
		hasDestination = false;
		startWaypoint = null;
		endWaypoint = null;
		lastWaypoint = null; 
				
		if(waypoints != null){
			waypoints.removeAll(waypoints);
		}
		waypoints = new ArrayList<Waypoint>();
		for(int i = 0; i<map[0].length; i=i+granularity){ 
			for(int j=0; j<map.length; j=j+granularity){
				if(isEmptySpace(j, i)){
					Waypoint n = new Waypoint(j, i);
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
				if(n1 != n2 && Math.abs(n1.x - n2.x) <= Math.sqrt(2*Math.pow(granularity,2)) && Math.abs(n1.y - n2.y) <= Math.sqrt(2*Math.pow(granularity,2))){
					// Check for static objects   
					if(isPath(n1, n2)){
						n1.getN().add(n2);
					}
				}
		    }
		}
		if(DEBUG){
			System.out.println("Path planner ready");
		}
	}
	public void reset(){ 
		
		if(DEBUG){
			System.out.println("In DEBUG mode");
			explored = new ArrayList<Waypoint>(); 
		}
		
		hasOrigin = false;
		hasDestination = false;
		startWaypoint = null;
		endWaypoint = null;
		lastWaypoint = null; 
		for(Waypoint w : waypoints){
			w.cameFrom = null;
		}
		if(DEBUG){
			System.out.println("Path planner ready");
		}
	}
	
	public void removeNode(int x, int y){
		Waypoint rm = findClosestWaypoint(x, y);
		if(euclideanDistance(rm.getX(), rm.getY(), x, y) > granularity){
			return;
		}
		for(int i=x-granularity/2; i<=x+granularity/2; i++){
			for(int j=y-granularity/2; j<=y+granularity/2; j++){
				map[i][j] = 0b00000000; 
			}
		}
		waypoints.remove(rm);
		for(Waypoint w : waypoints){
			w.getN().remove(rm);
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
				pruneStoredPath();
				return;
			}
			current = current.cameFrom;
			storedPath.add(current);
		}
	}
	
	private void pruneStoredPath(){
		int checkA = storedPath.size()-1; 
		int checkB = 0;
		int k = 2;
		while(checkA-checkB > 1 || checkA > 1){
			boolean canSee = canSee(storedPath.get(checkA), storedPath.get(checkB));
			if(canSee){
//				System.out.println("I can see " + checkB + " from " + checkA);
				int j = storedPath.size()-1;
				for(int i = 0; i<(checkA-checkB-1); i++){
//					System.out.println("Deleting: " + (storedPath.size()-k));
					storedPath.remove(storedPath.size()-k);
//					System.out.println("Size of path: " + storedPath.size());
				}
				canSee = false;
				checkB = 0;
				k++;
				checkA = storedPath.size()-k+1;
//				System.out.println("checkA: " + checkA + " checkB: " + checkB);
			}
			else{
				checkB++; 
			}
//			System.out.println("Continue: " + (checkA-checkB > 1 || checkA > 1));
//			System.out.println("checkA: " + checkA + " checkB: " + checkB);
		}
	}
	public boolean canSee(Waypoint a, Waypoint b){
//		System.out.println("In cansee()...");
		Vector3 vs = new Vector3(b.x-a.x, b.y-a.y, 0);
		double d = euclideanDistance(vs.getX(), vs.getY(), 0, 0);
		Vector3 v = vs.divScalar((float) d);
//		System.out.println("In cansee()... Entering for loop");
		for(int i = 1; i<Math.ceil(d)+1; i++){
//			System.out.println("In cansee()... i= " + i);
//			System.out.println("Vx " + v.getX() + " Vy " + v.getY());
//			System.out.println("Ax " + a.x + " By " + a.y);
//			System.out.println("Vix " + v.multScalar(i).getX() + " Viy " + v.multScalar(i).getY());
			Vector3 c = Vector3.addVectors(new Vector3(a.x, a.y, 0), v.multScalar(i));  // v + 
//			System.out.println("Vx " + v.getX() + " Vy " + v.getY());
//			System.out.println("cx " + c.getX() + " cy " + c.getY());
//			if(b.x == (int)c.getX() && b.y == (int)c.getY()){
			if(Math.abs(b.x-c.getX()) < 4 && Math.abs(b.y -c.getY()) < 4){ // What is the correct number here?
//				System.out.println("In cansee()...returning true");
				return true;
			}
			byte col = map[(int) c.getX()][(int) c.getY()]; // .getPixel(checkX, checkY);
		    if(isOccupied(col)){
		    	return false;
		    }
		}
		return false;
	}
	
	private boolean isOccupied(byte col){
//		return (((col)&0xFF) < 100 || ((col>>8)&0xFF) < 100 || ((col>>16)&0xFF) < 100);
		return ((col & 0xFF) < 30);
	}
	
	public boolean findPath(){
		ArrayList<Waypoint> closedSet = new ArrayList<Waypoint>();
		ArrayList<Waypoint> openSet = new ArrayList<Waypoint>();
		startWaypoint.fScore = euclideanDistance(startWaypoint.x, startWaypoint.y,endWaypoint.x, endWaypoint.y);
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
		    for(Waypoint n : current.getN()){
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
		    byte c = map[checkX][checkY]; // .getPixel(checkX, checkY);
//		    if(((c)&0xFF) < 30 || ((c>>8)&0xFF) < 30 || ((c>>16)&0xFF) < 30){
		    if(isOccupied(c)){
		    	return false;
		    }
		}
		return true;
	}
	
	public ArrayList<Waypoint> getExplored(){
		return explored;
	}
		
	private boolean isEmptySpace(int j, int i){
		byte c = map[j][i]; // .getPixel(j, i); //get(j, i);
		if((c & 0xFF) > 230){ 
			return true;
		}
		return false;
	}
		  
	public Waypoint findClosestWaypoint(int x, int y){
		Waypoint closest = null;
		double distance = Double.MAX_VALUE;
		for(Waypoint n : waypoints){
			double dist = euclideanDistance(x, y, n.x, n.y);
		    if(dist < distance){
		    	distance = dist;
		        closest = n;
		    }
		}
		return closest;
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
		return closest;
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
	
	public void setPose(int x, int y){
		posX = x;
		posY = y;
		byte col = map[posX][posY]; // .getPixel(checkX, checkY);
	    if(isOccupied(col)){
	    	Waypoint w = findClosestWaypoint(posX, posY);
	    	posXAdj = w.getX();
	    	posYAdj = w.getY();
	    }
	    else{
	    	posXAdj = posX;
	    	posYAdj = posY; 
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