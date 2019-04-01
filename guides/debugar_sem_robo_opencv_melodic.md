# Como testar seu projeto sem um robô físico

## .bashrc

*Nota: **cancele** sua variável `ROS_MASTER_URI` que é usada para se comunicar com um robô físico* 

Este cancelamento é feito abrindo o arquivo `~/.bashrc` e colocando um `#` em frente à linha em que a variável é definida.

```bash
#export ROS_MASTER_URI="http://"$IPBerry":11311"
```
Lembre-se de que alterações no `.bashrc` só se tornam efetivas em **novos terminais**

## Instalações com apt-get

Instale pacotes relacionados com vídeo:

	sudo apt-get install ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-openni-camera  ros-melodic-usb-cam  ros-melodic-async-web-server-cpp

## Software de acesso à webcam

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


### Executar

Para executar, **versão resumida**:

	rosrun cv_camera cv_camera_node

Depois de executar em geral **nada acontece**. A imagem aparece como um tópico ROS com o nome `/cv_camera/image_raw` e você precisa abrir usando `rqt_image_view`.

**Atenção**: Se aparecer um erro como o que vem a seguir, não há problema. Você pode seguir adiante desde que não esteja trabalhando com o modelo geométrico da câmera

	[ INFO] [1554125696.719076263, 17.953000000]: Unable to open camera calibration file [/home/borg/.ros/camera_info/camera.yaml]
	[ WARN] [1554125696.719251480, 17.953000000]: Camera calibration file /home/borg/.ros/camera_info/camera.yaml not found.

Este erro é solucionado se você [calibrar a câmera](calibrar_camera.md). Etapa necessária por exemplo se você precisar trabalhar com marcadores de realidade aumentada.


Para executar -  **versão completa se precisar mudar parâmetros**:

	rosrun cv_camera cv_camera_node _image_width:=800  _image_height:=600 _device_id:=0 _camera_info_url:=https://raw.githubusercontent.com/Insper/robot19/master/guides/head_camera.yaml

	rosrun cv_camera cv_camera_node  _device_id:=0 _camera_info_url:=https://raw.githubusercontent.com/Insper/robot19/master/guides/head_camera.yaml


## Mudanças no código

As mudanças necessárias no código se resumem a uma linha: Troque o nome do tópico `/raspicam_node/image/compressed` por `/cv_camera/image_raw/compressed`

Passará a ficar assim:

```python
recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
```

Para evitar que o código tenha que ser mudado, veja a dica abaixo


## Para republicar o tópico em /kamera

Na simulação com robô, a câmera se chama `/camera/rgb/image_raw` , no robô se chama `raspicam_node/image`  e neste jeito de emular, `/cv_camera/image_raw`.

Nossa sugestão é que renomeio o tópico válido para `/kamera` , assim terá mais facilidade durante os testes.

Para renomear a *webcam*

	rosparam set cv_camera/device_id 0
	rosparam set cv_camera/cv_cap_prop_frame_width 640
	rosparam set cv_camera/cv_cap_prop_frame_height  480
	rosrun cv_camera cv_camera_node
	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera

Para renomear a câmera simulada do Gazebo

	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera

Para renomear a câmera da Raspberry

	rosrun topic_tools relay /raspicam_node/image/compressed /kamera

Note que, após fazer *relay*, pode ser que o `/kamera` não funcione no `rqt_image_view` , mas ** vai funcionar no seu código**





