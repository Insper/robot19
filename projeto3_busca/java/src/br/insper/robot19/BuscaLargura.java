package br.insper.robot19;

import java.util.*;


/**
 * Classe que implementa o algoritmo de busca
 * @author antonio
 *
 */
public class BuscaLargura {
	
	private Block start = null;
	private Block end = null;
	private GridMap map = null;
	
	private Queue<Node> border;
	
	/**
	 * Construtor
	 * @param map
	 * @param start - ponto inicial
	 * @param end - ponto final
	 */
	public BuscaLargura(GridMap map, Block start, Block end) {
		this.map = map;
		this.start = start;
		this.end = end;
	}
	
	/**
	 * Método que realiza a busca
	 * @return
	 */
	public Node buscar() {

		Node root = new Node(start, null, null, 0, 0);
		border = new LinkedList<Node>();
		border.add(root);
		HashSet<Block> listablocos = new HashSet<Block>();

		
		while(!border.isEmpty()) {

			Node node = border.remove();
			Block atual = node.getValue();
			listablocos.add(atual);
			
			if(atual.row == end.row && atual.col == end.col) {
				return node;
			
			} else for(RobotAction acao : RobotAction.values()) {
				
				Block proximo = map.nextBlock(atual, acao);
				
				if(proximo != null && proximo.type != BlockType.WALL && !listablocos.contains(proximo)) {
					Node novoNode = new Node(proximo, node, acao, proximo.type.cost, 0);
					border.add(novoNode);
				}
			}
		}
		return null;
	}
	
	/**
	 * Resolve o problema com base em busca, realizando
	 * o backtracking após chegar ao estado final
	 * 
	 * @return A solução encontrada
	 */
	public RobotAction[] resolver() {
		
		// Encontra a solução através da busca
		Node destino = buscar();
		if(destino == null) {
			return null;
		}
		
		//Faz o backtracking para recuperar o caminho percorrido
		Node atual = destino;
		Deque<RobotAction> caminho = new LinkedList<RobotAction>();
		while(atual.getAction() != null) {
			caminho.addFirst(atual.getAction());
			atual = atual.getParent();
		}
		return caminho.toArray(new RobotAction[caminho.size()]);
	}
}
