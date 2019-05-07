# Atenção

Se você usa um `drive` bootável fornecido pela equipe da sala 404, todos os softwares já estão prontos e instalados.

## Instalar ROS Melodic no Ubuntu 18.04 com suporte a Turtlebot3

Vamos figurar o sources.list para que o apt (sistema de pacotes do Ubuntu) encontre o ROS.

     sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

Configurar as keys criptográficas que assinam o software do ROS:

    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

Atualizar a lista de pacotes:

    sudo apt update

Instalar softwares do ROS que usaremos no curso: 

    sudo apt-get install ros-melodic-joy ros-melodic-teleop-tools  ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch  ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view  ros-melodic-navigation ros-melodic-interactive-markers  ros-melodic-kobuki-ftdi ros-melodic-ar-track-alvar-msgs ros-melodic-teleop-tools python-rosinstall

Mais alguns softwares:

    sudo apt install     python-rosinstall-generator ros-melodic-ar-track-alvar ros-melodic-ar-track-alvar-dbgsym ros-melodic-ar-track-alvar-msgs ros-melodic-desktop-full ros-melodic-eigen-conversions ros-melodic-eigen-conversions-dbgsym ros-melodic-eigen-stl-containers ros-melodic-gscam ros-melodic-opencv-apps ros-melodic-opencv-apps-dbgsym ros-melodic-parrot-arsdk ros-melodic-turtlebot3-applications-msgs ros-melodic-turtlebot3-fake ros-melodic-turtlebot3-fake-dbgsym ros-melodic-turtlebot3-gazebo ros-melodic-turtlebot3-gazebo-dbgsym ros-melodic-turtlebot3-msgs ros-melodic-turtlebot3-simulations ros-melodic-vision-opencv ros-melodic-turtlebot3-msgs ros-melodic-ar-track-alvar ros-melodic-turtlebot3-applications-msgs

Instalar software para lidar com *stream* de vídeo vindo do robô:

    sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-* gstreamer1.0-libav* gstreamer1.0-plugins*

Configurar o ambiente de trabalho do ROS, que por padrão fica em `~/catkin_ws/src` :


    source /opt/ros/melodic/setup.bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws
    catkin_make

Vamos fazer o Linux enxergar pacotes que temos no nosso diretório do ROS:

    echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc



# Controlar o Turtle

Instalar biblioteca que reconhece marcadores de RA:

    sudo apt install ros-melodic-turtlebot3-msgs ros-melodic-ar-track-alvar ros-melodic-turtlebot3-applications-msgs


Vamos obter o software que falta para controle do Turtlebot clonando os repositórios da ROBOTIS:

    cd ~/catkin_ws/src
    git clone https://github.com/ros-teleop/teleop_twist_keyboard.git    
    git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_applications.git  


    cd ~/catkin_ws/
    catkin_make

Adicionar a configuração do robô ao `.bashrc`

    echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc


## Verificando se o simulador funciona

Tentar ver se o simulador do robô funciona:

    roslaunch turtlebot3_gazebo turtlebot3_world.launch

Em outro terminal

    rqt_image_view

Ainda em outro:

    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


Fechar o simulador



Adicionar estas linhas ao `.bashrc` para ser mais fácil de rodar o Turtlebot

    export IPBerry=192.168.0.110

**Atenção**: use o endereco  IP correto que aparece na tela do seu robô Turtlebot

    export ROS_MASTER_URI="http://"$IPBerry":11311"
    export ROS_IP=`hostname -I`
    export TURTLEBOT3_MODEL=waffle_pi

**Nota:** No simulador aconselhamos usar `export TURTLEBOT3_MODEL=waffle_pi`, mas para o robô físico é necessário ter `export TURTLEBOT3_MODEL=burger` . Esta sugestão é porque o *burger* simulado não tem câmera, e o *Waffle_Pi* tem. 

Para iniciar a conexão:

    roslaunch turtlebot3_bringup turtlebot3_remote.launch

Para ver a câmera:

    rqt_image_view

Para teclar:

    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
