package br.insper.robot19.vrep;

import java.util.ArrayList;
import java.util.List;

import br.insper.robot19.BlockType;
import br.insper.robot19.GridMap;
import coppelia.FloatW;
import coppelia.FloatWA;
import coppelia.IntWA;
import coppelia.StringWA;
import coppelia.remoteApi;

/**
 * Classe que representa o cenário do robô
 * @author Antonio Selvatici
 *
 */
public class VrepWorld {

	private static final String ROBOT_NAME_REGEX = "^Pioneer_p3dx[0-9_]*$";
	private static final String DOOR_NAME_REGEX = "^slidingDoor[0-9_]*$";
	private static final String FLOOR_NAME_REGEX = "^ResizableFloor[0-9_]+$";
	private static final String METALFLOOR_NAME_REGEX = "^[0-9cmX]*MetallicFloor[0-9]*$";
	private static final String SANDBUMP_NAME_REGEX = "Bump[0-9]*$";
	private static final String WALL_REGEX = "^[0-9cm]*cmHighWall[0-9]+cm[0-9]*$";
		
	private final remoteApi vrep;
	private final int clientId;
	private final int[] objHandles;
	private final String[] objNames;
	
	/**
	 * Construtor da classe
	 * @param vrep instância do cliente da API remota
	 * @param clientId ID do cliente
	 */
	VrepWorld(remoteApi vrep, int clientId) {
		this.vrep = vrep;
		this.clientId = clientId;
		
		// Now try to retrieve data in a blocking fashion (i.e. a service call):
        IntWA objectHandles = new IntWA(1);
        StringWA objectNames = new StringWA(1);
        int ret= vrep.simxGetObjectGroupData(clientId, remoteApi.sim_appobj_object_type, 0 /*names*/, objectHandles, new IntWA(1), new FloatWA(1), objectNames, remoteApi.simx_opmode_blocking);
        if (ret==remoteApi.simx_return_ok) {
            System.out.format("Number of objects in the scene: %d\n",objectHandles.getArray().length);
            objHandles = objectHandles.getArray();
            objNames = objectNames.getArray();
        } else { 
            throw new RuntimeException(String.format("Remote API function call returned with error code: %d\n",ret));
        }		
	}

	/**
	 * Constroi um mapa de grade a partir do cenário atual do VREP.
	 *
	 * As dimensões do mapa resultam da divisão do tamanho do modelo
	 * "Resizable_Floor" pelo tamanho da célula
	 *
	 * Os elementos que são capturados do cenário simulado são:
	 * - A posição do robô é mapeada como a posição inicial da busca
	 * - A posição do objeto Sliding door é mapeada como a posição de destino
	 * - As paredes do tipo "240, 20 ou 80 cm High Wall" são mapeadas como obstáculos
	 * - A superfície de metal "5x5m Metallic Floor" é mapeada como chão de metal
	 * - A superfície do tipo Bump é mapeada como chão de areia
	 * @param cellSize o tamanho da célula do mapa, em metros
	 * @return o mapa de grade construído
	 */
	public GridMap buildMap(float cellSize) {
		
		float[] floorCoords = null;
		float[] robotCoords = null;
		float[] doorCoords = null;
		List<float[]> wallCoordList = new ArrayList<float[]>();
		List<float[]> sandfloorCoordList = new ArrayList<float[]>();
		List<float[]> metalfloorCoordList = new ArrayList<float[]>();
		
		//Procura as coordenadas dos objetos comparando o nome
		for (int i = 0; i < objNames.length; i ++) {
			if(objNames[i].matches(ROBOT_NAME_REGEX)) {
				//Get the robot position as the initial position for planning
				FloatWA pos = new FloatWA(3);
				vrep.simxGetObjectPosition(clientId, objHandles[i], -1, pos, remoteApi.simx_opmode_blocking);
				robotCoords = pos.getArray();
			} else if(objNames[i].matches(DOOR_NAME_REGEX)) {
				//Get the robot position as the initial position for planning
				FloatWA pos = new FloatWA(3);
				vrep.simxGetObjectPosition(clientId, objHandles[i], -1, pos, remoteApi.simx_opmode_blocking);
				doorCoords = pos.getArray();
			} else if(objNames[i].matches(WALL_REGEX)) {
				//object i is a wall, discover min and max coordinates
				wallCoordList.add(xMinYMinXMaxYMax(objHandles[i]));
			} else if (objNames[i].matches(FLOOR_NAME_REGEX)) {
				floorCoords = xMinYMinXMaxYMax(objHandles[i]);
			} else if (objNames[i].matches(SANDBUMP_NAME_REGEX)) {
				sandfloorCoordList.add(xMinYMinXMaxYMax(objHandles[i]));
			} else if (objNames[i].matches(METALFLOOR_NAME_REGEX)) {
				metalfloorCoordList.add(xMinYMinXMaxYMax(objHandles[i]));
			}
		}
			
		//Calcula tamanho total do mapa através do objeto "Resizable_Floor"
		if(floorCoords != null) {			
			float sizex = floorCoords[2]-floorCoords[0];
			float sizey = floorCoords[3]-floorCoords[1];

			GridMap map = new GridMap(
					(int)Math.floor(sizey/cellSize), 
					(int)Math.floor(sizex/cellSize), 
					cellSize, floorCoords[0], floorCoords[1]);

			//Seta a posição inicial a partir da posiçao do robô
			if(robotCoords != null) {
				int[] rowColIni = map.gridRowCol(robotCoords[0], robotCoords[1]);
				map.setStart(rowColIni[0], rowColIni[1]);
			}
			
			//Seta a posição inicial a partir da posiçao da porta onde o robô deve passar
			if(doorCoords != null) {
				int[] rowColFim = map.gridRowCol(doorCoords[0], doorCoords[1]);
				map.setGoal(rowColFim[0], rowColFim[1]);
			}
			
			//Desenha o chão metálico no grid de células
			for (float[] coords : metalfloorCoordList) {
				map.drawFloor(BlockType.METAL, coords[0], coords[1], coords[2], coords[3]);
			}
			
			//Desenha o chão de areia no grid de células
			for (float[] coords : sandfloorCoordList) {
				map.drawFloor(BlockType.SAND, coords[0], coords[1], coords[2], coords[3]);
			}

			//Desenha as paredes no grid de células - tem que ser a etapa final
			for (float[] coords : wallCoordList) {
				map.drawWall(coords[0], coords[1], coords[2], coords[3]);
			}
			return map;
		}
		
		return null;
	}

	/**
	 * TODO: compensate object rotation
	 * @param objHandle
	 * @return
	 */
	private float[] xMinYMinXMaxYMax(int objHandle) {
		FloatW minx = new FloatW(.0f);
		FloatW miny = new FloatW(.0f);
		FloatW maxx = new FloatW(.0f);
		FloatW maxy = new FloatW(.0f);
		
		FloatWA pos = new FloatWA(3);
		vrep.simxGetObjectPosition(clientId, objHandle, -1, pos, remoteApi.simx_opmode_blocking);
		
		FloatWA ang = new FloatWA(3);
		vrep.simxGetObjectOrientation(clientId, objHandle, -1, ang, remoteApi.simx_opmode_blocking);
		
		vrep.simxGetObjectFloatParameter(clientId, objHandle, remoteApi.sim_objfloatparam_modelbbox_min_x, minx, remoteApi.simx_opmode_blocking);
		vrep.simxGetObjectFloatParameter(clientId, objHandle, remoteApi.sim_objfloatparam_modelbbox_min_y, miny, remoteApi.simx_opmode_blocking);
		vrep.simxGetObjectFloatParameter(clientId, objHandle, remoteApi.sim_objfloatparam_modelbbox_max_x, maxx, remoteApi.simx_opmode_blocking);
		vrep.simxGetObjectFloatParameter(clientId, objHandle, remoteApi.sim_objfloatparam_modelbbox_max_y, maxy, remoteApi.simx_opmode_blocking);
		
		//Mudança de coordenadas
		float[] minXY = {minx.getValue(), miny.getValue()};
		float[] maxXY = {maxx.getValue(), maxy.getValue()};
		float[] minXYNew = {0, 0};
		float[] maxXYNew = {0, 0};
		
		coordTransform(minXY, ang.getArray(), pos.getArray(), minXYNew);
		coordTransform(maxXY, ang.getArray(), pos.getArray(), maxXYNew);
		
		float[] coords = { minXYNew[0], minXYNew[1], maxXYNew[0],maxXYNew[1] };
		return coords;
	}
	
	//TODO use a full 3d coord transform
	private void coordTransform(float[] posIni, float[] ang, float transl[], float[] posFin) {
		
		posFin[0] = (float) (posIni[0]*Math.cos(ang[2]) - posIni[1]*Math.sin(ang[2]) + transl[0]);
		posFin[1] = (float) (posIni[0]*Math.sin(ang[2]) + posIni[1]*Math.cos(ang[2]) + transl[1]);
		if(posIni.length > 2 && posFin.length > 2) posFin[2] = posIni[2];
	}
	
	
	
	
}
