# Como testar seu projeto sem um robô físico

*Nota: **cancele** sua variável `ROS_MASTER_URI` que é usada para se comunicar com um robô físico* 

	#export ROS_MASTER_URI="http://"$IPBerry":11311"


Instale pacotes relacionados com vídeo:

	sudo apt-get install ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-openni-camera  ros-melodic-usb-cam  ros-melodic-async-web-server-cpp

Baixe um software que permite à OpenCV abrir diretamente a câmera:

	cd ~/catkin_ws/src
	git clone https://github.com/OTL/cv_camera

Em seguida compile:

	cd ~/catkin_ws/

	catkin_make

## Abra o simulador

Você precisa de um robô simulado. Vamos disparar o simulador:

	roslaunch turtlebot3_gazebo turtlebot3_world.launch 


## Para rodar emulando a câmera com OpenCV

Defina os parâmetros

	rosparam set cv_camera/device_id 0
	rosparam set cv_camera/cv_cap_prop_frame_width 640
	rosparam set cv_camera/cv_cap_prop_frame_height  480


Nota: se você precisar trabalhar com marcadores de realidade aumentada precisará [calibrar a câmera](calibrar_camera.md).


Para executar:

	rosrun cv_camera cv_camera_node


Para executar: versão completa:

rosrun cv_camera cv_camera_node _image_width:=800  _image_height:=600 _device_id:=0 _camera_info_url:=https://raw.githubusercontent.com/Insper/robot19/master/guides/head_camera.yaml

rosrun cv_camera cv_camera_node  _device_id:=0 _camera_info_url:=https://raw.githubusercontent.com/Insper/robot19/master/guides/head_camera.yaml






## Mudanças no código

As mudanças necessárias no código se resumem a uma linha: Troque o nome do tópico `/raspicam_node/image/compressed` por `/cv_camera/image_raw/compressed`

Passará a ficar assim:

	recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)

Para evitar que o código tenha que ser mudado, veja a dica abaixo


## Para republicar o tópico em /kamera

Na simulação com robô, a câmera se chama `/camera/rgb/image_raw` , no robô se chama `raspicam_node/image`  e neste jeito de emular, `/cv_camera/image_raw`.

Nossa sugestão é que renomeio o tópico válido para `/kamera` , assim terá mais facilidade durante os testes.

Para renomear a *webcam*

	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera

Para renomear a câmera simulada do Gazebo

	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera /cam

Para renomear a câmera da Raspberry

	rosrun topic_tools relay /raspicam_node/image/compressed /kamera /cam




