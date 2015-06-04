package org.munoz.indoorassistednavigation.pathplanner;

import java.util.ArrayList;
import org.munoz.mathlib.Vector3;


public class PathPlanner {
	int[][] map; 
	int granularity;
	private ArrayList<Node> nodes;
	Node start = null;
	Node end = null;
	int clearAreaThreshold = 40; // TODO: Remove magic numbers
	public ArrayList<Node> storedPath;
	public int posX;
	public int posY;
	public boolean hasStartNode = false;
	public boolean hasEndNode = false;
	public Node lastEndNode = null;
	public int currentFloor; 
	
	public enum PlannerState{
		READY_TO_LISTEN, MAP_DISPLACEMENT, SETTING_START, SETTING_END, NO_TASK_SELECTED, READ_LIST_OF_TARGETS, GOING_TO_DESTINATION, ARRIVED
	}
	
	public PlannerState pState = null;
	
	public PathPlanner(int[][] m){ //Image m){
		map = m;
		granularity = 8; // TODO: remove magic numbers 
		createNodes();
	}
	public PathPlanner(int [][]m, int gran){
		map = m;
		granularity = gran;
		createNodes();
	}

	public void createNodes(){
		hasStartNode = false;
		hasEndNode = false;
		if(nodes != null){
			nodes.removeAll(nodes);
		}
		nodes = new ArrayList<Node>();
		for(int i = 0; i<map[0].length; i=i+granularity){  // .getHeight()
			for(int j=0; j<map.length; j=j+granularity){
				if(isEmptySpace(j, i)){
					Node n = new Node(j, i);
					n.gScore = Double.MAX_VALUE; // TODO: This should be 0 at the beginning. 
					nodes.add(n);
				}
		    }
		}    
//		println(nodes.size());
		// Neighbors
		for(Node n1 : nodes){
			for(Node n2 : nodes){
				// TODO: Granularity * 2??? 
				if(n1 != n2 && Math.abs(n1.x - n2.x) <= granularity && Math.abs(n1.y - n2.y) <= granularity){
					// Check for walls   
					if(isPath(n1, n2)){
						n1.getNeighbors().add(n2);
					}
				}
		    }
		}
	}
	public ArrayList<Node> getNodes() {
		return nodes;
	}
	

	public boolean setStartNode(int x, int y){
		start = findClosestNode(x, y);
		hasStartNode = true;
		return (start != null);
	}
	
	public boolean setEndNode(int x, int y){
		end = findClosestNode(x, y);
		hasEndNode = true;
		lastEndNode = end;
		return (end != null);
	}
	
	public Node getStartNode(){
		return start;
	}
	public Node getEndNode(){
		return end;
	}
	// TODO: Remove?
//	private void clearArea(int x, int y){
//		color c = get(x, y);
//		color white = #ffffff;
//		
//		stroke(white);
//
//		int[][] openPixels = new int[width][height];
//		openPixels[x][y] = 1;
//		ArrayList<Pixel> pixelBag = new ArrayList<Pixel>();
//		pixelBag.add(new Pixel(x, y));   
//		while(!pixelBag.isEmpty()){
//			println(pixelBag.size());
//		    Pixel p = pixelBag.get(0); 
//		    x = p.x;
//		    y = p.y;
//		    openPixels[x][y] = 1; //, white);
//		    pixelBag.remove(p); 
//		    println(pixelBag.size());
//		    // add neighbors;
//		    if(pixelBag.size() < 400){
//		      matchingPixels(pixelBag, openPixels, x, y, c, true);
//		      //break;
//		    }
//		    else{
//		      matchingPixels(pixelBag, openPixels, x, y, c, false);
//		    }
//		  } 
//		  
//		  println("painting");
//		  for(int i=0; i<width; i++){
//		    for(int j=0; j<height; j++){
//		      if(openPixels[i][j] == 1){
//		        set(i, j, white);
//		      }
//		    }
//		  }
//		  noFill();
//		  activateClearAreaSelector = false;
//		//  save("./images/imageProc.png");
//		//  map = loadImage("./images/imageProc.png");
//		}
//		void matchingPixels(ArrayList<Pixel> o, int[][] a, int x, int y, color c, boolean add){
//		//  color d = c;
//		  for(int i=0; i<3; i++){
//		    for(int j=0; j<3; j++){
//		      int xn = x - 1 + i; 
//		      int yn = y - 1 + j;
//		      color n = get(xn, yn);
//		      //println(red(n));
//		      //println(red(c));
//		      //println(Math.abs(red(n) - red(c)));
//		      if(xn == x && yn == y){
//		        continue;
//		      }
//		      if((Math.abs(red(n) - red(c)) < clearAreaThreshold) &&
//		        (Math.abs(blue(n) - blue(c)) < clearAreaThreshold) &&
//		        (Math.abs(green(n) - green(c)) < clearAreaThreshold) &&
//		        (a[xn][yn] != 1) && add){
//		        o.add(new Pixel(xn, yn)); 
//		      }
//		      //else if((a[xn][yn] != 1) && red(n) > 245 && blue(n) > 245 && green(n) > 245){
//		      //  o.add(new Pixel(xn, yn)); 
//		      //}
//		    }
//		  } 
//		}
	
	public int getGranularity(){
		return granularity;
	}

	private void showPath(){
		boolean drawingPath = true;
		Node current = end;
		if(end == null){
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
		storedPath = new ArrayList<Node>();
		if(end == null)
			return;
		Node current = end;
		storedPath.add(end);
		while(true){
			if(current.cameFrom == null){
				return;
			}
			current = current.cameFrom;
			storedPath.add(current);
		}
	}
	
	public boolean findPath(){
		ArrayList<Node> closedSet = new ArrayList<Node>();
		ArrayList<Node> openSet = new ArrayList<Node>();
		double g = 0;
		start.fScore = g + euclideanDistance(start.x, end.x, start.y, end.y);
		openSet.add(start);
		// TODO: REALLY inefficient. FIX;
		while(!openSet.isEmpty()){
			Node current = lowestFScore(openSet);
		    if(current == end){
				System.out.println("Found path"); 
		    	return true; //  
		    }
		    closedSet.add(current);
		    openSet.remove(current);
		    // find best neighbor; 
//		      current = lowestFScore(current.neighbors); ???
		    for(Node n : current.getNeighbors()){
		    	if(closedSet.contains(n)){
		    		continue;
		    	}
		    	double tentativeG = g + euclideanDistance(current.x, current.y, n.x, n.y);
		    	if(!openSet.contains(n) || tentativeG < n.gScore){
		    		n.cameFrom = current;
		    		n.gScore = tentativeG;
		    		n.fScore = n.gScore + euclideanDistance(n.x, n.y, end.x, end.y);
		    		if(!openSet.contains(n)){
		    			openSet.add(n);
		    		}
		    	}     
		    }  
		}
		System.out.println("Couldn't find path"); // TODO: Handle this. s
		return false;
	}
	
	private Node lowestFScore(ArrayList<Node> nodeList){
		double minF = Double.MAX_VALUE;
		Node lowest = null;
		for(Node n : nodeList){
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
	
	// TODO: Remove?
//	void redrawOriginalImage(){
//		  image(map, 0, 0);
//		}
	
	private boolean isPath(Node a, Node b){
		int checkX = a.x;
		int checkY = a.y;
		while(checkX != b.x || checkY != b.y){
			if(checkX < b.x){
				checkX++;
		    }
		    else{
		    	checkX--;
		    }
		    if(checkY < b.y){
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
		    if(((c)&0xFF) < 30 || ((c>>8)&0xFF) < 30 || ((c>>16)&0xFF) < 30){
		    	return false;
		    }
		}
		return true;
	}
		
	private boolean isEmptySpace(int j, int i){
		int c = map[j][i]; // .getPixel(j, i); //get(j, i);
		int r = (c)&0xFF;
		int g = (c>>8)&0xFF;
		int b = (c>>16)&0xFF;
		if(r > 230 && g > 230 && b > 230){
			return true;
		}
		return false;
	}
		  
	public Node findClosestNode(int x, int y){
		Node closest = null;
		double distance = Double.MAX_VALUE;//map.getWidth();//width;
//		Log.e("Nodes", "" + nodes.size());
		for(Node n : nodes){
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
	
	public void markNode(String label){
		Node n = findClosestNode(posX, posY);
		if(n!= null){
			n.setLabel(label);
		}
		else{
			// TODO: Handle this error
		}
	}
	public static double getAngleNextNode(Node origin, double thetaRadians, Node nextNode) {
		Vector3 v1 = new Vector3(thetaRadians, 2);
		Vector3 v2 = new Vector3(nextNode.getX()-origin.getX(), nextNode.getY()-origin.getY(), 0);
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