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

	@Override
	public int hashCode() {
		return row*1000000 + col;
	}

	@Override
	public boolean equals(Object obj) {

		if(obj == null) return false;
		else if(obj == this) return true;
		else if(obj instanceof Block) {
			Block other = (Block) obj;
			return other.row == row && other.col == col;
		} else return false;
	}
}
