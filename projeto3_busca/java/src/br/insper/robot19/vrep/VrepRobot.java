package br.insper.robot19.vrep;

import br.insper.robot19.RobotAction;
import coppelia.FloatWA;
import coppelia.IntW;
import coppelia.remoteApi;

/**
 * Classe que representa o robô simulado no VREP
 * 
 * @author Antonio Selvatici
 *
 */
public class VrepRobot {
	
	//Parameters updated for Pioneer P3DX
	private static final float wheelRadius = 0.09751f;
	private static final float maxSpeed = 0.80f; // m/s
	private static final float kLinear = 2.0f; // m/s
	private static final float kAngular = 0.005f; // rad/s
	
	private static final String robotName = "Pioneer_p3dx";
	private static final String leftMotorName = "Pioneer_p3dx_leftMotor";
	private static final String rightMotorName = "Pioneer_p3dx_rightMotor";
	
	private final remoteApi vrep;
	private final int clientId;
	private final int robotHandle;
	private final int leftMotorHandle;
	private final int rightMotorHandle;
	
	/**
	 * Construtor, faz a conexão com o simulador e 
	 * captura os handles para o robô e seus motores
	 * @param vrep a instância da API
	 * @param clientId o ID do cliente remoto
	 */
	VrepRobot(remoteApi vrep, int clientId) {
		
		this.vrep = vrep;
		this.clientId = clientId;
		
        IntW handle = new IntW(0);
        int ret = vrep.simxGetObjectHandle(clientId, robotName, handle, remoteApi.simx_opmode_blocking);
        if (ret != remoteApi.simx_return_ok) throw new RuntimeException("Não foi possível encontrar o objeto " + robotName);
        robotHandle = handle.getValue();

        ret = vrep.simxGetObjectHandle(clientId, leftMotorName, handle, remoteApi.simx_opmode_blocking);
        if (ret != remoteApi.simx_return_ok) throw new RuntimeException("Não foi possível encontrar o objeto " + leftMotorName);
        leftMotorHandle = handle.getValue();

        ret = vrep.simxGetObjectHandle(clientId, rightMotorName, handle, remoteApi.simx_opmode_blocking);
        if (ret != remoteApi.simx_return_ok) throw new RuntimeException("Não foi possível encontrar o objeto " + rightMotorName);
        rightMotorHandle = handle.getValue();
        
        System.out.println("Robô simulado inicializado.");
	}

	/**
	 * Retorna a posição (x, y) do robô simulado em coordendas
	 * absolutas do mapa, em metros.
	 * @return um array [x, y] contendo a posição do robô
	 */
	public float[] getPosition() {
		
		int ret = vrep.simxSetJointTargetVelocity(clientId, leftMotorHandle, 0.0f, remoteApi.simx_opmode_oneshot);
		ret = vrep.simxSetJointTargetVelocity(clientId, rightMotorHandle, 0.0f, remoteApi.simx_opmode_oneshot);
		
		FloatWA position = new FloatWA(3);	
		ret = vrep.simxGetObjectPosition(clientId, robotHandle, -1, position, remoteApi.simx_opmode_blocking);
		if(ret == remoteApi.simx_return_ok) {
			float[] pos = { position.getArray()[0], position.getArray()[1]};
			return pos;
		}
		return null;
	}

	/**
	 * Envia os sinal de controle para o robô simulado
	 * executar comandos preespecificados
	 * @param action o comando {@link RobotAction} a ser executado
	 * @param magnitude a magnitude do comando, ou seja, o quanto
	 *                     o robô deve se mover na direção especificada, em metros.
	 */
	public void execute(RobotAction action, float magnitude) {
		switch(action) {
		case DOWN:
			turnTo(-90.0);
			moveForward(magnitude);
			break;
		case LEFT:
			turnTo(180.0);
			moveForward(magnitude);
			break;
		case RIGHT:
			turnTo(0.0);
			moveForward(magnitude);
			break;
		case UP:
			turnTo(90.0);
			moveForward(magnitude);
			break;
		}		 
	}
	
	/**
	 * Move o robô para frente usando um controle proporcional dos motores
	 * @param meters o quanto o robô deverá se mover, em metros
	 */
	public void moveForward(float meters) {
		
		//Pega a posiçõa atual do robô
		FloatWA position = new FloatWA(3);
		FloatWA positionIni = new FloatWA(3);
		int ret = vrep.simxGetObjectPosition(clientId, robotHandle, -1, positionIni, remoteApi.simx_opmode_blocking);
		if(ret != remoteApi.simx_return_ok) {
			System.out.println("Robô não encontrado no simulador");
			return;
		}
		
		// Itera até que o robô chegue próximo o suficiente ao objetivo 
		float dist = 0.0f;
		while ( Math.abs(dist - meters) > 0.01 ) {
				
			//Calcula a velocidade angular a ser enviada para os motores com controle proporcional
			float targetSpeed = Math.max(-maxSpeed, Math.min(kLinear * (meters - dist), maxSpeed));
			float targetAngularSpeed = targetSpeed/wheelRadius;
			
			// Envia a velocidade de giro aos motores
			ret = vrep.simxSetJointTargetVelocity(clientId, leftMotorHandle, targetAngularSpeed, remoteApi.simx_opmode_oneshot);
			ret = vrep.simxSetJointTargetVelocity(clientId, rightMotorHandle, targetAngularSpeed, remoteApi.simx_opmode_oneshot);
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				throw new RuntimeException(e);
			}
			
			// Pega a nova posição do robô 
			ret = vrep.simxGetObjectPosition(clientId, robotHandle, -1, position, remoteApi.simx_opmode_blocking);
			if(ret != remoteApi.simx_return_ok) {
				System.out.println("Robô não encontrado no simulador");
				return;
			}

			// Calcula a nova distância percorrida
			float deltax = position.getArray()[0] - positionIni.getArray()[0];
			float deltay = position.getArray()[1] - positionIni.getArray()[1];
			dist = (float) Math.sqrt(deltax*deltax + deltay*deltay);
		}
	}
	
	
	/**
	 * Gira o robô para o ângulo fornecido, com controle proporcional dos motores
	 * @param degrees - ângulo final desejado de em graus
	 */
	public void turnTo(double degrees) {
		
		//Pega a orientação atual do robô
		FloatWA orient = new FloatWA(3);
		int ret = vrep.simxGetObjectOrientation(clientId, robotHandle, -1, orient, remoteApi.simx_opmode_blocking);
		if(ret != remoteApi.simx_return_ok) {
			System.out.println("Robô não encontrado no simulador");
			return;
		}
		float thetaIni = this.getAngle(orient.getArray());
				
		// Itera até que o robô chegue próximo o suficiente ao objetivo 
		float theta = thetaIni;
		while ( Math.abs(theta - degrees) > 0.5 ) { // max error in degrees 
			
			//Calcula a velocidade angular a ser enviada para os motores com controle proporcional
			float targetSpeed = (float) Math.max(-maxSpeed, Math.min(kAngular * (degrees - theta), maxSpeed));
			float targetAngularSpeed = targetSpeed/wheelRadius;
			
			// Envia a velocidade de giro aos motores
			ret = vrep.simxSetJointTargetVelocity(clientId, leftMotorHandle, -targetAngularSpeed, remoteApi.simx_opmode_oneshot);
			ret = vrep.simxSetJointTargetVelocity(clientId, rightMotorHandle, targetAngularSpeed, remoteApi.simx_opmode_oneshot);
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				throw new RuntimeException(e);
			}
			
			// Pega a nova posição do robô 
			ret = vrep.simxGetObjectOrientation(clientId, robotHandle, -1, orient, remoteApi.simx_opmode_blocking);
			if(ret != remoteApi.simx_return_ok) {
				System.out.println("Robô não encontrado no simulador");
				return;
			}
			theta = this.getAngle(orient.getArray());
		}
	}	
	
	private float getAngle(float[] eulerAngles) {
		return  (float)((eulerAngles[2] > Math.PI ? eulerAngles[2] - Math.PI*2  : eulerAngles[2])*180.0/Math.PI); 
	}	
}
