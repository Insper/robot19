
# SLAM no Turtlebot 3

Estes são os comandos necessários para executar o SLAM (*simultaneous localization and mapping*) com o software do Turtlebot3.

Cada comando precisa ser dado num terminal diferente.

Primeiro precisamos definir qual Turtlebot usar na simulação. A versão *Waffle* é interessante porque já vem com a câmera (este comando deve ser repetido em todo terminal, ou adicionado ao final do ~/.bashrc).

    export TURTLEBOT3_MODEL=waffle_pi

Depois iniciamos o ambiente virtual de simulação:

    roslaunch turtlebot3_gazebo turtlebot3_world.launch

Depois o serviço de SLAM

    roslaunch turtlebot3_slam turtlebot3_slam.launch

Agora um terminal que nos permita controlar o robô com as teclas:

    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Finalmente o RVIZ - painel de instrumentação:

    rosrun rviz rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_slam.rviz
