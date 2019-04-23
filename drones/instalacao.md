
## Instalação do Bebop_Autonomy

Link do tutorial original (que fui adaptando)

[https://bebop-autonomy.readthedocs.io/en/latest/installation.html](https://bebop-autonomy.readthedocs.io/en/latest/installation.html)

Crie o diretório `~/catkin_ws` 

Dentro do `~/catkin_ws`, foi emitido o comando `catkin_init_workspace`

Depois: 

    cd catkin_ws/src

Em seguida: 

    git clone https://github.com/AutonomyLab/bebop_autonomy.git

Depois

    rosdep update

No diretório

    cd    catkin_ws


Rodar

    rosdep install --from-paths src -i

depois, para compilar:

    cd ~/catkin_ws/src

    catkin_make