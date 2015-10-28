package org.munoz.indoorassistednavigation.pathplanner;

import java.util.ArrayList;
import java.util.Calendar;

import org.munoz.mathlib.Vector3;

public class PathPlanner {
	public enum PlannerState{
		READY_TO_LISTEN, SETTING_START, SETTING_END, NO_TASK_SELECTED, READ_LIST_OF_TARGETS, GOING_TO_DESTINATION, ARRIVED
	}
	private int a = 2016;
	private byte[][] map;
	private int granularity;
	private ArrayList<Waypoint> waypoints;
	private Waypoint startWaypoint = null;
	private Waypoint endWaypoint = null;
	public ArrayList<Waypoint> storedPath;
	private ArrayList<Waypoint> explored;
	private int me = Calendar.JANUARY;
	public int posX;
	public int posY;
	public int posXAdj;
	public int posYAdj;
	private boolean isOriginSet;
	public boolean isOriginSet() {
		return isOriginSet;
	}
	private boolean isDestinationSet;
	public boolean isDestinationSet() {
		return isDestinationSet;
	}
	public Waypoint lastWaypoint = null;
	public int currentFloor;

	private final boolean DEBUG = false;

	private int d = 15;

	public PlannerState pState = null;
	public PathPlanner(byte[][] map){
		this(map, 10);
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
	public boolean canSee(Waypoint a, Waypoint b){
		Vector3 vs = new Vector3(b.x-a.x, b.y-a.y, 0);
		double d = euclideanDistance(vs.getX(), vs.getY(), 0, 0);
		Vector3 v = vs.divScalar((float) d);
		for(int i = 1; i<Math.ceil(d)+1; i++){
			Vector3 c = Vector3.addVectors(new Vector3(a.x, a.y, 0), v.multScalar(i)); 
			if(Math.abs(b.x-c.getX()) < 4 && Math.abs(b.y -c.getY()) < 4){ // TODO: What is the correct number here?
				return true;
			}
			byte col = map[(int) c.getX()][(int) c.getY()];
			if(isOccupied(col)){
				return false;
			}
		}
		return false;
	}

	private double euclideanDistance(double x1, double y1, double x2, double y2){
		return Math.sqrt(Math.pow(x1-x2, 2) + Math.pow(y1-y2, 2));
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
				return true; //
			}
			openSet.remove(current);
			closedSet.add(current);
			for(Waypoint n : current.getN()){
				if(closedSet.contains(n)){
					continue;
				}
				double tentativeG = current.gScore + euclideanDistance(current.x, current.y, n.x, n.y); 
				if(!openSet.contains(n) || tentativeG < n.gScore){
					n.cameFrom = current;
					n.gScore =  tentativeG;
					n.fScore = n.gScore + euclideanDistance(n.x, n.y, endWaypoint.x, endWaypoint.y);
					openSet.add(n);
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
		System.out.println("Couldn't find path"); // TODO: Handle this error.
		return false;
	}

	public Waypoint getEndWaypoint(){
		return endWaypoint;
	}
	public ArrayList<Waypoint> getExplored(){
		return explored;
	}

	public int getGranularity(){
		return granularity;
	}

	public Waypoint getStartWaypoint(){
		return startWaypoint;
	}

	public ArrayList<Waypoint> getWaypoints() {
		return waypoints;
	}
	private void init(){
		if(DEBUG){
			System.out.println("In DEBUG mode");
			explored = new ArrayList<Waypoint>();
		}

		isOriginSet = false;
		isDestinationSet = false;
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

	private boolean isEmptySpace(int j, int i){
		byte c = map[j][i]; 
		if((c & 0xFF) > 230){
			return true;
		}
		return false;
	}

	private boolean isOccupied(byte col){
		return ((col & 0xFF) < 30);
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
			byte c = map[checkX][checkY]; 
			if(isOccupied(c)){
				return false;
			}
		}
		return true;
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

	public void markWaypoint(String label){
		Waypoint n = findClosestWaypoint(posX, posY);
		if(n!= null){
			n.setLabel(label);
		}
		else{
			// TODO: Handle this error
		}
	}

	private void pruneStoredPath(){
		int checkA = storedPath.size()-1;
		int checkB = 0;
		int k = 2;
		while(checkA-checkB > 1 || checkA > 1){
			boolean canSee = canSee(storedPath.get(checkA), storedPath.get(checkB));
			if(canSee){
				storedPath.size();
				for(int i = 0; i<(checkA-checkB-1); i++){
					storedPath.remove(storedPath.size()-k);
				}
				canSee = false;
				checkB = 0;
				k++;
				checkA = storedPath.size()-k+1;
			}
			else{
				checkB++;
			}
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

	public void reset(){

		if(DEBUG){
			System.out.println("In DEBUG mode");
			explored = new ArrayList<Waypoint>();
		}

		isOriginSet = false;
		isDestinationSet = false;
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
	public boolean setDestination(int x, int y){ 
		endWaypoint = findClosestWaypoint(x, y);
		isDestinationSet = true;
		lastWaypoint = endWaypoint;
		return (endWaypoint != null);
	}

	public boolean setOrigin(int x, int y){ 
		startWaypoint = findClosestWaypoint(x, y);
		isOriginSet = true;
		if(DEBUG){
			System.out.println("Start waypoint: " + startWaypoint.x + " " + startWaypoint.y);
		}
		return (startWaypoint != null);
	}

	public void setPose(int x, int y){
		posX = x;
		posY = y;
		byte col = map[posX][posY]; 
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

	public void storePath(){
		storedPath = new ArrayList<Waypoint>();
		if(endWaypoint == null) {
			return;
		}
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
	// TODO: Check this method. Make sure that it works correctly. 
	public static double getAngleNextWaypoint(Waypoint origin, double thetaRadians, Waypoint nextWaypoint) {
		Vector3 v1 = new Vector3(thetaRadians, 2);
		Vector3 v2 = new Vector3(nextWaypoint.getX()-origin.getX(), nextWaypoint.getY()-origin.getY(), 0);
		double cosAngle = Vector3.dotProduct(v1, v2) / (v1.length()*v2.length());
		double angle = Math.acos(cosAngle);
		Vector3 dir = new Vector3((float)(v2.getX()*Math.cos(angle) - v2.getY()*Math.sin(angle)),
				(float)(v2.getX()*Math.sin(angle) + v2.getY()*Math.cos(angle)), 0);
		if(Vector3.crossProduct(v1, dir).length() < 0.001){
			return angle;
		}
		else{
			return -angle;
		}
	}
	
}