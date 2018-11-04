// TODO: Add Licence. 

package org.munoz.indoorassistednavigation.pathplanner;

import java.util.ArrayList;
import java.util.Calendar;

import org.munoz.mathlib.Vector3;

public class PathPlanner {
	public enum PlannerState{
		READY_TO_LISTEN, SETTING_START, SETTING_END, NO_TASK_SELECTED, READ_LIST_OF_TARGETS, GOING_TO_DESTINATION, ARRIVED
	}
	private byte[][] map;
	private int granularity;
	private ArrayList<Waypoint> waypoints;
	private ArrayList<Waypoint> removedW;
	private ArrayList<Waypoint> removedWN;
	private ArrayList<Pixel> toReset;
	private Waypoint startWaypoint = null;
	private Waypoint endWaypoint = null;
	public ArrayList<Waypoint> storedPath;
	private ArrayList<Waypoint> explored;
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
	private final static int DEFAULT_GRANULARITY = 10;
	private final static int MIN_GRANULARITY = 2; // 7; 
	private final static int MAX_GRANULARITY = 30; 
	private final static int MAX_HEAT = 30;
	private final static int LAPLACIAN_INDEX = 4; 
	private final static int REACHABILITY_INDEX = 230; 
	
	public PlannerState pState = null;
	public PathPlanner(byte[][] map){
		this(map, DEFAULT_GRANULARITY);
	}

	public PathPlanner(byte [][]m, int gran){
		map = m;
		granularity = gran;
		if(granularity < MIN_GRANULARITY){
			granularity = MIN_GRANULARITY;
		}
		else if(granularity > MAX_GRANULARITY){
			granularity = MAX_GRANULARITY;
		}
		init();
	}
	public boolean canSee(Waypoint a, Waypoint b){
		if(a == null || b == null){			
			return false;
		}
		Vector3 vs = new Vector3(b.x-a.x, b.y-a.y, 0);
		double d = euclideanDistance(vs.getX(), vs.getY(), 0, 0);
		Vector3 v = vs.divScalar((float) d);
		for(int i = 1; i<Math.ceil(d)+1; i++){
			Vector3 c = Vector3.addVectors(new Vector3(a.x, a.y, 0), v.multScalar(i)); 
			if(Math.abs(b.x-c.getX()) < LAPLACIAN_INDEX && Math.abs(b.y -c.getY()) < LAPLACIAN_INDEX){ 
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
		double distance = Double.MAX_VALUE;
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
		while(!openSet.isEmpty()){
			Waypoint current = lowestFScore(openSet);
			if(current == endWaypoint){
				return true; 
			}
			openSet.remove(current);
			closedSet.add(current);
			for(Waypoint n : current.getN()){
				if(closedSet.contains(n)){
					continue;
				}
				double tentativeG = current.gScore + euclideanDistance(current.x, current.y, n.x, n.y); 
				if(!openSet.contains(n) || tentativeG < n.gScore){
					n.from = current;
					n.gScore =  tentativeG;
					n.fScore = n.gScore + euclideanDistance(n.x, n.y, endWaypoint.x, endWaypoint.y);
					openSet.add(n);
				}
			}
		}
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
		isOriginSet = false;
		isDestinationSet = false;
		startWaypoint = null;
		endWaypoint = null;
		lastWaypoint = null;

		if(waypoints != null){
			waypoints.removeAll(waypoints);
		}
		waypoints = new ArrayList<Waypoint>();
		removedW = new ArrayList<Waypoint>(); 
		removedWN = new ArrayList<Waypoint>();
		toReset = new ArrayList<Pixel>(); 
		for(int i = 0; i<map[0].length; i=i+granularity){
			for(int j=0; j<map.length; j=j+granularity){
				if(isEmptySpace(j, i)){
					Waypoint n = new Waypoint(j, i);
					waypoints.add(n);
				}
			}
		}
		for(Waypoint n1 : waypoints){
			for(Waypoint n2 : waypoints){
				if(n1 != n2 && Math.abs(n1.x - n2.x) <= Math.sqrt(2)*granularity && Math.abs(n1.y - n2.y) <= Math.sqrt(2)*granularity){
					if(isPath(n1, n2)){
						n1.getN().add(n2);
					}
				}
			}
		}
	}

	private boolean isEmptySpace(int j, int i){
		byte c = map[j][i]; 
		if((c & 0xFF) > REACHABILITY_INDEX){
			return true;
		}
		return false;
	}

	private boolean isOccupied(byte col){
		return ((col & 0xFF) < MAX_HEAT);
	}

	private boolean isPath(Waypoint a, Waypoint b){
		if(a == null || b == null){
			return false; 
		}
		int checker1 = a.x;
		int checker2 = a.y;
		while(checker1 != b.x || checker2 != b.y){
			if(checker1 == b.x){
			}
			else if(checker1 < b.x){
				checker1++;
			}
			else{
				checker1--;
			}
			if(checker2 == b.y){
			}
			else if(checker2 < b.y){
				checker2++;
			}
			else{
				checker2--;
			}
			if(checker2 < 0 || checker2 > map[0].length || checker1 < 0 || checker1 > map.length){
				return false;
			}
			byte c = map[checker1][checker2]; 
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
	}

	private void pruneStoredPath(){
		int checkA = storedPath.size()-1;
		int checkB = 0;
		int k = 2;
		while(checkA-checkB > 1 || checkA > 1){
			// TODO: Remove. I have to add this because of error in fullreset. 
//			System.out.println(storedPath.size() + " " + checkA + " " + checkB);
//			if(storedPath.size() == checkB){
//				break;
//			}
			// END REMOVE!!
			
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
//	static int countRemoved = 0; 
//	static int countRemInN = 0; 
	
	// http://stackoverflow.com/questions/471962/how-do-determine-if-a-polygon-is-complex-convex-nonconvex
//	given p[k], p[k+1], p[k+2] each with coordinates x, y:
//		 dx1 = x[k+1]-x[k]
//		 dy1 = y[k+1]-y[k]
//		 dx2 = x[k+2]-x[k+1]
//		 dy2 = y[k+2]-y[k+1]
//		 zcrossproduct = dx1*dy2 - dy1*dx2
	
	
	private void cornerMaker(){
		// Check that path is greater than 2. 
		ArrayList<Integer> toRemove = new ArrayList<Integer>(); 
		ArrayList<Pixel> toAdd = new ArrayList<Pixel>();
		for(int i=1; i<storedPath.size()-2; i++){

			Waypoint k = storedPath.get(i-1);
			Waypoint k1 = storedPath.get(i);
			Waypoint k2 = storedPath.get(i+1);
			Waypoint k3 = storedPath.get(i+2);
			if(euclideanDistance(k1.getX(), k1.getY(), k2.getX(), k2.getY()) <= Math.sqrt(2)*granularity*2){
				if(k.getX() <= k1.getX() && k.getY() >= k1.getY() && k2.getX() <= k3.getX() && k2.getY() <= k3.getY()){
					toRemove.add(i);
					toRemove.add(i+1);
					toAdd.add(new Pixel(k1.getX(), k2.getY()));
					System.out.println("Removed!");
				}
				
//				int dx1 = k1.getX() - k.getX();
//				int dy1 = k1.getY() - k.getY();
//				int dx2 = k2.getX() - k1.getX();
//				int dy2 = k2.getY() - k1.getY();
//				double zcross = dx1*dy2 - dy1*dx2;
//				if(zcross > 0){
//					Waypoint n = findClosestWaypoint(k2.getX()-granularity-2, k1.getY()-granularity-2);
//					if(n != null && n.getX() == k2.getX()-granularity && n.getY() == k1.getY()-granularity){
//						storedPath.add(i, n);
//						storedPath.remove(k2);
//					}
//				}
//				else if(zcross < 0){
//					Waypoint n = findClosestWaypoint(k2.getX()+granularity-2, k1.getY()-granularity-2);
//					if(n != null && n.getX() == k2.getX()-granularity && n.getY() == k1.getY()-granularity){
//						storedPath.add(i, n);
//						storedPath.remove(k2);
//					}
//				}
//				System.out.println(zcross);
			}
		}
		int countAdd = 0;
		boolean removedFirst = false; 
		for(int i=0; i<toRemove.size(); i+=2){
			if(removedFirst){
				
			}
			else{
				Waypoint n = findClosestWaypoint(toAdd.get(countAdd).x, toAdd.get(countAdd).y);
				if(n != null && n.getX() == toAdd.get(countAdd).x && n.getY() == toAdd.get(countAdd).y){
					int remI = toRemove.get(i);
					storedPath.remove(remI);
					storedPath.remove(toRemove.get(i+1));
					storedPath.add(remI+(countAdd*2), n);
				}
				countAdd++; 
			}
		}

		

//		pruneStoredPath();
	}
	
	public void removeNode(int x, int y){
		Waypoint rm = findClosestWaypoint(x, y);
		if(euclideanDistance(rm.getX(), rm.getY(), x, y) > granularity){
			return;
		}
		for(int i=x-granularity/2; i<=x+granularity/2; i++){
			for(int j=y-granularity/2; j<=y+granularity/2; j++){
				map[i][j] = 0b00000000;
				
				toReset.add(new Pixel(i, j));
				
			}
		}
		removedW.add(rm);
		
		// TODO: Remove
//		System.out.println("Removed: " + rm.getX() + " " + rm.getY());
//		countRemoved++;
		
		// TODO: Find a way to simplify this
		for(Waypoint w : rm.getN()){ // waypoints){
			if(w.getN().remove(rm)){
				w.getRMN().add(rm);
				if(!removedWN.contains(w)){
					removedWN.add(w);
//					System.out.println("Removed N: " + w.getX() + " " + w.getY());	
//					countRemInN++;
				}
			}
		}
		waypoints.remove(rm);
	}

	public void reset(){
		isOriginSet = false;
		isDestinationSet = false;
		startWaypoint = null;
		endWaypoint = null;
		lastWaypoint = null;
		for(Waypoint w : waypoints){
			w.from = null;
		}
	}
	
	public void resetFull(){
		
//		countRemoved = 0; 
//		System.out.println("CountRMN: " + countRemInN);	
//		countRemInN = 0; 
//		System.out.println(removedW.size() + " " + removedWN.size());
		
		for(Pixel p : toReset){
			map[p.x][p.y] = 0b01111111; // TODO: Check this. 
		}
		
		for(Waypoint w : removedW){
			waypoints.add(w);
//			System.out.println("Restored: " + w.getX() + " " + w.getY());	
//			countRemoved++; 
			
		}
		
//		for(Waypoint n1 : removedW){
//			for(Waypoint n2 : removedWN){
//				if(n1 != n2 && Math.abs(n1.x - n2.x) <= Math.sqrt(2)*granularity && Math.abs(n1.y - n2.y) <= Math.sqrt(2*Math.pow(granularity,2))){
//					if(isPath(n1, n2)){
//						n1.getN().add(n2);
//					}
//				}
//			}
//		}
		removedW.clear();
		
		for(Waypoint w : removedWN){
//			System.out.println("Restoring n in " + w.getX() + " " + w.getY());
			for(Waypoint r : w.getRMN()){
				w.getN().add(r);
//				System.out.println("Restored: " + r.getX() + " " + r.getY() + " in " + w.getX() + " " + w.getY());			
//				System.out.println(countRemInN++);
			}
			w.getRMN().clear();
		}
		removedWN.clear();
		toReset.clear();
		reset(); 
		
		
		
////		System.out.println("CountRM: " + countRemoved);	
////		countRemoved = 0; 
////		System.out.println("CountRMN: " + countRemInN);	
////		countRemInN = 0; 
//		
//		// TODO: REMOVE
//		System.out.println("REMOVE THIS");
//		int counter = 0; 
//		for(Waypoint w : waypoints){
//			for(Waypoint n : w.getN()){
//				counter++;
//			}
//		}
//		System.out.println("Total n: " + counter);		
//		for(Waypoint w : waypoints){
//			for(Waypoint n : w.getN()){
//				if(!n.getN().contains(w)){
//					System.out.println("Error");
//				}
//			}
//		}
//		System.out.println("HERE");
		
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
			if(current.from == null){
				pruneStoredPath();
				//cornerMaker(); // JP: Bing likes to copy everything. Rename. 
				return;
			}
			current = current.from;
			storedPath.add(current);
		}
	}
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
