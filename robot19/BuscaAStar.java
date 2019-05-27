package br.insper.robot19;

import java.awt.*;
import java.util.*;

public class BuscaAStar {

    private Block start = null;
    private Block end = null;
    private GridMap map = null;
    private PriorityQueue<Node> border;
    private GraficoMapAStar canvas;
    private int seq;

    /**
     * Construtor
     * @param map
     * @param start - ponto inicial
     * @param end - ponto final
     */
    public BuscaAStar(GridMap map, Block start, Block end) {
        this.map = map;
        this.start = start;
        this.end = end;
        this.canvas = new GraficoMapAStar(map);
        canvas.desenha();
    }

    /**
     * Método que realiza a busca
     * @return
     */
    public Node buscar() {

        HashSet<Block> stack = new HashSet<>();

        Node root = new Node(start, null, null, 0);

//Limpa a fronteira e insere o nó raiz

        Comparator<Node> comparator = new ComparadorAStar();
        border = new PriorityQueue<Node>(comparator);
        border.add(root);


        while(!border.isEmpty()) {

            Node node = border.remove();
            Block atual = node.getValue();

            if(!stack.contains(atual)){
                stack.add(atual);
                if(atual.row == end.row && atual.col == end.col) {
                    canvas.drawBlock(atual, Color.BLACK);
                    return node;
                } else for(RobotAction acao : RobotAction.values()) {
                    Block proximo = map.nextBlock(atual, acao);
                    if(proximo != null && proximo.type != BlockType.WALL) {
                        Node novoNode = new Node(proximo, node, acao, proximo.type.cost);
                        novoNode.setH((float)Math.sqrt(Math.pow(end.col - proximo.col, 2) + Math.pow(end.row - proximo.row, 2)));
                        border.add(novoNode);
                        if (!stack.contains(proximo)){
                            canvas.drawBlock(proximo, Color.lightGray);
                        }
                    }
                }
                canvas.drawBlock(atual, Color.BLACK);
                canvas.saveFile("AStar" + seq + ".png");
                seq++;
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
        RobotAction[] solucao = caminho.toArray(new RobotAction[caminho.size()]);
        canvas.setAndDrawSolucao(solucao);
        return solucao;

    }
}