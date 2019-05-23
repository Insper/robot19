package br.insper.robot19;

public class Node {
	private Block value;
	private Node parent;
	private RobotAction action;
	private float pathCost;
	private float heuristica;

	public Node (Block value, Node parent, RobotAction action, float cost, float heuristica) {
		this.value = value;
		this.parent = parent;
		this.action = action;
		this.pathCost = parent == null ? 0 : parent.getPathCost() + cost;
		this.heuristica = heuristica;
	}


	public Block getValue() {
		return value;
	}
	
	public Node getParent() {
		return parent;
	}
	
	public RobotAction getAction() {
		return action;
	}
	
	public float getPathCost() {
		return pathCost;
	}

	public float getHeuristica() {
		return heuristica;
	}

}
