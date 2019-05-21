package br.insper.robot19;

public class Block {
	
	public final int row;
	public final int col;
	public final BlockType type;
	
	public Block(int i, int j, BlockType type) {
		this.row = i;
		this.col = j;
		this.type = type;
	}


	
	
}
