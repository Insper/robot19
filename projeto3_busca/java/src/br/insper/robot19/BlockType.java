package br.insper.robot19;

import java.lang.Float;

public enum BlockType {
	
	FREE(1.0f, '.'),
	METAL(3.0f, '3'),
	SAND(10.f, 'X'),
	WALL(Float.POSITIVE_INFINITY, '#');
	
	public final float cost;
	public final char symbol;
	private BlockType(float cost, char symbol) {
		this.cost = cost;
		this.symbol = symbol;
	}

}
