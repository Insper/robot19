# Atividade 2 do projeto - OpenCV no robô

No ROS a OpenCV trabalha com base em eventos. Esta atividade permite que você estude isso mais a fundo

## Atividades


### 0. Setup

Baixe e rode o *script* para instalar a OpenCV nova também no Python 2 (que é o default do ROS)

```bash
wget https://raw.githubusercontent.com/Insper/robot19/master/guides/instalar_opencv_python2.sh
chmod a+x instalar_opencv_python2.sh
sudo ./instalar_opencv_python2.sh
```

Aguarde cerca de 10 minutos


### 1. Programa ROS 

Faça um clone de [https://github.com/insper/robot19/](https://github.com/insper/robot19/) **dentro** de sua pasta `catkin_ws/src`.

    cd ~/catkin_ws/src
    git clone https://github.com/insper/robot19/
    cd ~/catkin_ws
    catkin_make


Estude o código de `cor.py`. Você pode começar executando este programa. Primeiro **[conecte num robô](https://github.com/Insper/robot19/blob/master/guides/bringup_turtlebot.md)** para poder testar

Rode num terminal o comando para que o tópico de câmera tenha um repetidor (relay) chamado `/kameras`

    rosrun topic_tools relay /raspicam_node/image/compressed /kamera


E em outro terminal

    rosrun exemplos_python cor.py

Se necessário, **pegue uma caixa vermelha** emprestada com os técnicos do laboratório.


**Agora, faça:**

Modique este para que o robô centralize sempre o objeto vermelho. Depois que o robô estiver centralizado no vermelho, deve segui-lo

Crie [seu próprio projeto](https://github.com/Insper/robot19/blob/master/guides/projeto_rospython.md), não trabalhe na pasta clonada do professor.


### 2. Usando a simulação 

Leia [este guia de como rodar seu código OpenCV sem um robô físico](https://github.com/Insper/robot19/blob/master/guides/debugar_sem_robo_opencv_melodic.md).

Abra o simulador de Turtlebot

Em seguida rode o mesmo código que você fez acima com o simulador e a webcam.



