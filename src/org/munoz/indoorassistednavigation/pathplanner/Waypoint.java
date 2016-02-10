package org.munoz.indoorassistednavigation.pathplanner;

import java.util.ArrayList;

public class Waypoint {
//	public static int waypoints; 
	int x;
	int y;
	private ArrayList<Waypoint> n = new ArrayList<Waypoint>();
	private ArrayList<Waypoint> rmN = new ArrayList<Waypoint>();
	private boolean inArea; 
	
	double fScore; 
	double gScore;
	Waypoint from;
	String label; 
	public Waypoint(int j, int i){
		x = j;
		y = i;
		label = null;
//		waypoints++; 
	}
	
	public int getX(){
		return x;
	}
	public int getY(){
		return y;
	}
	public void setInArea(boolean add){
		inArea = add; 
	}
	public boolean getInArea(){
		return inArea;
	}
	
	public ArrayList<Waypoint> getN(){
		return n;
	}
	
	public ArrayList<Waypoint> getRMN(){
		return rmN;
	}
	
	public String getLabel(){
		return label;
	}
	
	public void setLabel(String l){
		label = l;
	}
}
