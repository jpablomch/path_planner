package org.munoz.indoorassistednavigation.pathplanner;

import java.util.ArrayList;

public class Waypoint {
	int x;
	int y;
	private ArrayList<Waypoint> neighbors = new ArrayList<Waypoint>();
	double fScore; 
	double gScore;
	Waypoint cameFrom;
	String label; 
	public Waypoint(int j, int i){
		x = j;
		y = i;
		label = null;
	}
	
	public int getX(){
		return x;
	}
	public int getY(){
		return y;
	}
	
	public ArrayList<Waypoint> getNeighbors(){
		return neighbors;
	}
	
	public String getLabel(){
		return label;
	}
	
	public void setLabel(String l){
		label = l;
	}
	
	// TODO: Comment this out for release
//	public double getF(){
//		return fScore;
//	}
//	public double getG(){
//		return gScore;
//	}
}