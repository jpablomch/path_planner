package org.munoz.indoorassistednavigation.pathplanner;

import java.util.ArrayList;

public class Node {
	int x;
	int y;
	private ArrayList<Node> neighbors = new ArrayList<Node>();
	double fScore; 
	double gScore;
	Node cameFrom;
	String label; 
	public Node(int j, int i){
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
	
	public ArrayList<Node> getNeighbors(){
		return neighbors;
	}
	
	public String getLabel(){
		return label;
	}
	
	public void setLabel(String l){
		label = l;
	}
	
}