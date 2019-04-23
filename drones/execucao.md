


## Como conectar no drone se você já tem o bebop_autonomy instalado

Antes de se conectar, certifique-se de que a linha a seguir faz parte do arquivo  `.bashrc` 

    source ~/catkin_ws/devel/setup.bash

### Pré-requisitos

**1.** Certifique-se de que as variáveis `ROS_MASTER_URI` e `ROS_IP` estão desabilitadas no `~/.bashrc`

**2.** Anote o  número de série do drone antes de ligar.  O nome da rede em que você vai se conectar usa os 6 últimos caracteres do número serial. Para saber o número serial é preciso **remover a bateria** antes de iniciar o drone.

**3.** Conecte-se ao *access point* do *drone* via *wi-fi*.

**4.** Verifique se a conexão está funcionado:

    ping 192.168.42.1

### Conexão

Abrir um novo terminal. ara conectar com o drone 

    roslaunch bebop_driver bebop_node.launch

Para abrir a imagem:

    rqt_image_view 

Para visualizar os tópicos:

    rostopic list
    rqt_graph

Para fazer o drone decolar via terminal:

    rostopic pub --once bebop/takeoff std_msgs/Empty

Para fazer o drone pousar, via terminal (é aconselhável deixá-lo perto do solo)


    rostopic pub --once bebop/land std_msgs/Empty

Para controlar o drone via teleop de teclado:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=bebop/cmd_vel




## Projetos demo de alunos:

O que rodar:

[https://github.com/PhelipeMuller/PetDrone](https://github.com/PhelipeMuller/PetDrone)


Como eu rodei:

    cd ~/catkin_ws/src
    git clone https://github.com/PhelipeMuller/PetDrone
    cd PetDrone/
    cd projeto/
    python takeoffnland.py 


[https://github.com/decoejz/robotica_p3](https://github.com/decoejz/robotica_p3)

Como eu rodei:

    cd ~/catkin_ws/src
    git clone https://github.com/decoejz/robotica_p3
    cd robotica_p3/
    cd scripts/
    python ./drone_drive.py 




### Exemplo interessante mesclando real e virtual

[Visual servoing](http://repositorio.upct.es/bitstream/handle/10317/5442/pfc6362.pdf?sequence=1)




