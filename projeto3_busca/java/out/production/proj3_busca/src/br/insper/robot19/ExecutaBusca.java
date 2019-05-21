package br.insper.robot19;

import java.io.IOException;

import br.insper.robot19.vrep.VrepRobot;
import br.insper.robot19.vrep.VrepSimulator;
import br.insper.robot19.vrep.VrepWorld;

public class ExecutaBusca {

	private static float cellSize = 0.5f; //m

	public static void main(String[] args) {
		
		//Carrega o arquivo a partir do arquivo	
		GridMap map;
		try {
			map = GridMap.fromFile("map_teste.txt");
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
		
		if(map == null) 
			throw new RuntimeException("O mapa discretizado não pode ser obtido a partir do modelo");
		
		//Imprimie o mapa em ASCII
		System.out.println("Mapa inicial:");
		System.out.println(map);
		
		//Realiza a busca
		int[] rowColIni = map.getStart();
		int[] rowColFim = map.getGoal();
		Block inicial = new Block(rowColIni[0], rowColIni[1], BlockType.FREE) ;
		Block alvo = new Block(rowColFim[0], rowColFim[1], BlockType.FREE) ;
		BuscaLargura busca = new BuscaLargura(map, inicial, alvo);
		RobotAction[] solucao = busca.resolver();
		
		//Mostra a solução
		if(solucao == null) {
			System.out.println("Nao foi encontrada solucao para o problema");
		} else {
			
			Block atual = inicial;
			System.out.print("Solução: ");
			for(RobotAction a : solucao) {
				System.out.print(", " + a);
				Block next = map.nextBlock(atual, a);
				map.setRoute(next.row, next.col);
				atual = next;
			}
			
			//Mostra o mapa com a rota encontrada
			System.out.println();
			System.out.println("Rota encontrada:");
			System.out.println(map);
		}
	}
}
