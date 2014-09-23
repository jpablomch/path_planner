package org.munoz.indoorassistednavigation.pathplanner;

import java.util.ArrayList;

public class Node {
	int x;
	int y;
	ArrayList<Node> neighbors = new ArrayList<Node>();
	double fScore; 
	double gScore;
	Node cameFrom;
	String label; 
	public Node(int j, int i){
		x = j;
		y = i;
		label = null;
	}
//	public void show(color c){ //(EntryPoint entryPoint, Color c) {
//		fill(c);//entryPoint.fill(c); //(c.getRed(), c.getGreen(), c.getBlue());
//		ellipse(x, y,  10,  10); //entryPoint.ellipse(x, y,  10,  10);
//	}
	
	public int getX(){
		return x;
	}
	public int getY(){
		return y;
	}
	
	public String getLabel(){
		return label;
	}
	
	public void setLabel(String l){
		label = l;
	}
	
}