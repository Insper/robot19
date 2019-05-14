package br.insper.robot19;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class GridMap {
	
	public static final float DEFAULT_CELL_SIZE = 1.0f;
	
	private final BlockType[][] grid;
	private final BlockStatus[][] path; 
	private final float cellSize;
	private final float minX;
	private final float minY;
	
	private int startRow = -1, startCol = -1;
	private int goalRow = -1, goalCol = -1;
	
	public static GridMap fromFile(String fileName) throws IOException {

		Path p = Paths.get(fileName);
		byte[] content = java.nio.file.Files.readAllBytes(p);
		return fromString(new String(content));
	}

	
	/**
	 * Constrói o mapa a partir de texto
	 * @param text
	 * @return
	 */
	public static GridMap fromString(String text) {
		
		List<List<BlockType>> gridBuilder = new ArrayList<List<BlockType>>();
		List<List<BlockStatus>> pathBuilder = new ArrayList<List<BlockStatus>>();
		
		List<BlockType> currLine = null;
		List<BlockStatus> currLineSt = null;
		char[] cells = text.toCharArray();
		boolean newLine = true;
		
		for (char cell : cells) {
			
			if(newLine) {
				currLine = new ArrayList<BlockType>();
				currLineSt = new ArrayList<BlockStatus>();
				gridBuilder.add(currLine);
				pathBuilder.add(currLineSt);
				newLine = false;
			}
			
			for(BlockStatus status : BlockStatus.values()) {
				if(cell == status.symbol) {
					currLine.add(BlockType.FREE);
					currLineSt.add(status);
					break;
				} 
			}
			
			for(BlockType type : BlockType.values()) {
				if(cell == type.symbol) {
					currLine.add(type);
					currLineSt.add(null);
					break;
				} 
			}
			if(cell == '\n') {
				newLine = true;
			}
		}
		
		if(gridBuilder.size() == 0) {
			return null;
		} else if(gridBuilder.get(gridBuilder.size()-1).isEmpty()) {
			gridBuilder.remove(gridBuilder.size()-1);
			pathBuilder.remove(pathBuilder.size()-1);
		}
		
		//Constrói e preenche o mapa
		GridMap map = new GridMap(gridBuilder.size(), gridBuilder.get(0).size(), DEFAULT_CELL_SIZE, 0, 0);
		for (int i = 0; i < map.grid.length; i++) {
			for(int j = 0; j < Math.min(map.grid[0].length, map.grid[i].length); j++) {
				map.grid[i][j] = gridBuilder.get(i).get(j);
				map.path[i][j] = pathBuilder.get(i).get(j);
				if(map.path[i][j] == BlockStatus.START) {
					map.setStart(i, j);
				}
				if(map.path[i][j] == BlockStatus.GOAL) {
					map.setGoal(i, j);
				}
			}
		}
		
		return map;
	}
	
	/**
	 * Construtor
	 * @param nRows
	 * @param nCols
	 * @param cellSize
	 * @param minX
	 * @param minY
	 */
	public GridMap(int nRows, int nCols, float cellSize, float minX, float minY) {
		this.grid = new BlockType[nRows][nCols];
		this.path = new BlockStatus[nRows][nCols];
		this.cellSize = cellSize;
		this.minX = minX;
		this.minY = minY;
		
		//Fill grid map
		for(BlockType[] row : grid) {
			Arrays.fill(row, BlockType.FREE);
		}
	}

	public void setType(int row, int col, BlockType type) {
		this.grid[row][col] = type;
	}
	
	public boolean isOccupied(int row, int col) {
		return this.grid[row][col] == BlockType.WALL;
	}
	
	public int[] getStart() {
		if(startRow < 0 || startCol < 0) {
			return null;
		}
		int[] coords = {startRow, startCol};
		return coords;
	}
	public void setStart(int row, int col) {

		if(row >= 0 && row < path.length && col >=0 && col <= path[0].length ) {
			//There can be only one...
			if(startCol > -1 && startRow > -1) {
				path[startRow][startCol] = null;
			}
			startRow = row; startCol = col;
			path[row][col] = BlockStatus.START;
		}
	}

	public int[] getGoal() {
		if(goalRow < 0 || goalCol < 0) {
			return null;
		}
		int[] coords = {goalRow, goalCol};
		return coords;
	}
	public void setGoal(int row, int col) {

		if(row >= 0 && row < path.length && col >=0 && col <= path[0].length ) {
			//There can be only one...
			if(goalRow > -1 && goalCol > -1) {
				path[goalRow][goalCol] = null;
			}
			goalRow = row; goalCol = col;
			path[row][col] = BlockStatus.GOAL;
		}
	}
	
	public void setRoute(int row, int col) {
		
		if(row >= 0 && row < path.length && col >=0 && col <= path[0].length 
				&& path[row][col] != BlockStatus.START && path[row][col] != BlockStatus.GOAL) {
			path[row][col] = BlockStatus.ROUTE;
		}		
	}

	/**
	 * Determina a posição do próximo bloco dada uma ação
	 * @param atual
	 * @param action
	 * @return o próximo bloco, ou null caso o robô saia do mapa
	 */
	public Block nextBlock(Block atual, RobotAction action) {
		int row = atual.row;
		int col = atual.col;
		
		switch(action) {
		case UP: row--; break;
		case DOWN: row++; break;
		case LEFT: col--; break;
		case RIGHT: col++; break;
		}
		
		if(row < 0 || row >= grid.length ||
				col < 0 || col >= grid[0].length) {
			return null;
		}
		
		return new Block(row, col, grid[row][col]);
	}
	
	/**
	 * 
	 * @param x
	 * @param y
	 * @return
	 */
	public int[] gridRowCol(float x, float y) {
		int row = (int) ((minY + grid.length*cellSize - y)/cellSize);
		int col = (int) ((x - minX)/cellSize);
		return new int[] {row, col};
	}
	
	/**
	 * 
	 * @param x0
	 * @param y0
	 * @param x1
	 * @param y1
	 */
	public void drawWall(float x0, float y0, float x1, float y1) {
		
		//Converte coordendas para grid cells
		int gridX0 = (int) ((x0 - minX)/cellSize);
		int gridX1 = (int) ((x1 - minX)/cellSize);
		float maxY = minY + grid.length*cellSize;
		int gridY0 = (int) ((maxY - y0)/cellSize);
		int gridY1 = (int) ((maxY - y1)/cellSize);
		
		if(Math.abs(x1-x0) > Math.abs(y1-y0)) {
			bresenhamX(gridX0, gridY0, gridX1, gridY1);
		} else {
			bresenhamY(gridX0, gridY0, gridX1, gridY1);
		}
	}
	
public void drawFloor(BlockType type, float x0, float y0, float x1, float y1) {
		
		//Converte coordendas para grid cells
		int gridX0 = (int) ((x0 - minX)/cellSize);
		int gridX1 = (int) ((x1 - minX)/cellSize);
		float maxY = minY + grid.length*cellSize;
		int gridY0 = (int) ((maxY - y0)/cellSize);
		int gridY1 = (int) ((maxY - y1)/cellSize);
		
		float leftX = Math.min(gridX0, gridX1);
		float upY = Math.min(gridY0, gridY1);
		float rightX = Math.max(gridX0, gridX1);
		float lowY = Math.max(gridY0, gridY1);
		
		for(int i = (int)Math.max(0, upY); i < Math.min(grid.length, lowY); i++) {
			for(int j = (int) Math.max(0, leftX); j < Math.min(grid[i].length, rightX); j++) {
				setType(i, j, type);
			}
		}
	}
	
	/*
	 * 
	 */
	private void bresenhamX(int x0, int y0, int x1, int y1) {
		
		float deltax = x1 - x0;
		float deltay = y1 - y0; 
		// Assume deltax != 0 (line is not vertical),
		// note that this division needs to be done in
		// a way that preserves the fractional part
		float deltaerr = Math.abs(deltay / deltax);
		float error = 0.0f; // No error at start
		
		int x = x0;
		int y = y0;
		for(; x >= 0 && y >=0 && x < grid[0].length && y < grid.length && x != x1;
				x += Math.signum(deltax)) 
		{ 
			setType(y, x, BlockType.WALL);
			error += deltaerr;
			if(error >= 0.5) {
				y += Math.signum(deltay);
				error -= 1.0;
			}
		}
		if(x == x1) setType(y, x, BlockType.WALL);
	}
	
	private void bresenhamY(int x0, int y0, int x1, int y1) {

		float deltax = x1 - x0;
		float deltay = y1 - y0; 
		// Assume deltay != 0 (line is not horizontal),
		// note that this division needs to be done in
		// a way that preserves the fractional part
		float deltaerr = Math.abs(deltax / deltay);
		float error = 0.0f; // No error at start
		
		int x = x0;
		int y = y0;
		for(; x >= 0 && y >=0 && x < grid[0].length && y < grid.length && y != y1;
				y += Math.signum(deltay)) 
		{ 
			setType(y, x, BlockType.WALL);
			error += deltaerr;
			if(error >= 0.5) {
				x += Math.signum(deltax);
				error -= 1.0;
			}
		}
		if(y == y1) setType(y, x, BlockType.WALL);
	}


	@Override
	public String toString() {

		StringBuilder sb = new StringBuilder();
		for(int i = 0; i < grid.length; i++) {
			for(int j = 0; j < grid[i].length; j++) {
				char c = path[i][j] == null ? grid[i][j].symbol : path[i][j].symbol;
				sb.append( c );
			} sb.append(System.lineSeparator());
		}
		return sb.toString();

	}

	/**
	 * Use for DRAWING purposes only
	 * @param i - the x coordinate of the queried block type
	 * @param j - the y coordinate of the queried block time
	 * @return the block type at position (i,j)
	 */
	BlockType getBlockType(int i, int j){
		return this.grid[i][j];
	}

	/**
	 *
	 * @return how many columns this grid has
	 */
	int getWidth(){
		int ret = 0;
		if (grid != null) {
			if (grid[0]!= null) {
				return grid[0].length;
			}
		}
		return ret;
	}

	/**
	 *
	 * @return how many rows this grid has
	 */
	int getHeight(){
		if (grid != null){
			return grid.length;
		}
		return 0;
	}
}
