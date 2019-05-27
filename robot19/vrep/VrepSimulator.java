// Make sure to have the server side running in V-REP: 
// in a child script of a V-REP scene, add following command
// to be executed just once, at simulation start:
//
// simRemoteApi.start(19999)
//
// then start simulation, and run this program.
//
// IMPORTANT: for each successful call to simxStart, there
// should be a corresponding call to simxFinish at the end!

package br.insper.robot19.vrep;

import coppelia.IntW;
import coppelia.remoteApi;

/**
 * Classe singleton que representa a conexão com o simulador VREP
 * 
 * @author Antonio Selvatici
 *
 */
public class VrepSimulator implements AutoCloseable {

	/**
	 * Definição da porta de conexão à API remota do VREP
	 */
	private static final int PORT = 19997;
		
	private static VrepSimulator instance = null;

	/**
	 * Método estático para a obtenção da instância do Singleton
	 * @return o objeto {@link VrepSimulator} instanciado
	 */
	public static VrepSimulator getInstance() {
		if(instance == null) {
			instance = new VrepSimulator(PORT);
		}
		return instance;
	}
	
	private final remoteApi vrep;
	private final int clientId;
		
	private VrepSimulator(int port) {
		System.out.println("Program started.");
 		vrep = new remoteApi();
        vrep.simxFinish(-1); // just in case, close all opened connections
        clientId = vrep.simxStart("127.0.0.1", port, true, true, 5000, 5);
        if (clientId == -1)
        	throw new RuntimeException("O simulador não pode ser contactado");
      
        System.out.println("Connected to remote API server");   
	}
	
	@Override
	public void close() throws Exception {
		// Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        IntW pingTime = new IntW(0);
        vrep.simxGetPingTime(clientId,pingTime);
        // Now close the connection to V-REP:   
        vrep.simxFinish(clientId);
	}
	
	/**
	 * Cria uma instância do robô
	 * @return a instância criada que faz a comunicação com o robô simulado
	 */
	public VrepRobot createRobot() {
		return new VrepRobot(vrep, clientId);
	}
	
	/**
	 * Cria uma instância do cenário simulado do robô
	 * @return objeto que faz a comunicação com o cenário
	 */
	public VrepWorld createWorld() {
		return new VrepWorld(vrep, clientId);
	}
	
}
