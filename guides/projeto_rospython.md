# Como criar um projeto ROS Python

Fontes:

[AGITR Capítulo 3 (também disponivel na biblioteca)](https://www.cse.sc.edu/~jokane/agitr/agitr-letter-pubsub.pdf)

[ROS Robot Programming Capítulo 7](http://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51)


Você deve trabalhar sempre num *workspace catkin ROS*.  Por enquanto encorajamos fortemente  trabalhar sempre no `/home/usuario/catkin_ws` . Se você usa a instalação *default* vai ser então `\home\borg\catkin_ws`.

Em Unix em geral o caracter `~` sempre aponta para o diretório atual. Portanto trabalharemos no `~/catkin_ws/src`


Abra o terminal com `Ctrl` `Alt` `T`

    cd ~/catkin_ws/src


    catkin_create_pkg meu_projeto std_msgs sensor_msgs geometry_msgs rospy roscpp


A saída deve ser:

    borg@ubuntu:~/catkin_ws$ cd cd ~/catkin_ws/src
    borg@ubuntu:~/catkin_ws/src$ catkin_create_pkg meu_projeto std_msgs sensor_msgs geometry_msgs rospy roscpp
    Created file meu_projeto/CMakeLists.txt
    Created file meu_projeto/package.xml
    Created folder meu_projeto/include/meu_projeto
    Created folder meu_projeto/src
    Successfully created files in /home/borg/catkin_ws/src/meu_projeto. Please adjust the values in package.xml.

Vamos criar um diretório para os programas Python:

    cd ~/catkin_ws/src/meu_projeto
    mkdir scripts

Vamos criar nosso primeiro script (vazio inicialmente)

    touch roda.py

Agora vamos torná-lo executável:
    chmod a+x roda.py

Em seguida vamos editá-lo. Sugirmos usar o `subl`, que precisa ser instalado caso não exista:

    sudo snap install --classic sublime-text
    subl roda.py

Dentro do editor cole o seguinte código para o `roda.py:

```python

#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3

v = 10  # Velocidade linear
w = 5  # Velocidade angular

if __name__ == "__main__":
    rospy.init_node("roda_exemplo")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    try:
        while not rospy.is_shutdown():
            vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
            pub.publish(vel)
            rospy.sleep(2.0)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
```
Agora salve seu código.

Em seguida, vamos rodar o `catkin_make` para compilar (se necessário quando o código é em C++ ou Java) e para que o *ROS* identifique o código que fizemos

Para compilar:

    cd ~/catkin_ws/
    catkin_make

### Execução:

Para executar, é muito importante que você esteja ou (a) [com o simulador aberto](simulador_ros.md) ou (b) com o robô real conectado.

Digite **num terminal novo**, criado depois de ter rodado o `catkin_make` acima:

    rosrun meu_projeto roda.py

Note que o ROS encontra automaticamente seus scripts. Você não precisa de um terminal que esteja no mesmo diretório.  **Entretanto**, se seu código Python usa recuros locais (imagens, arquivos, etc) você vai precisar estar num diretório específico para executá-lo. 

Neste caso, para mudar de diretório faça:

    roscd meu_projeto/scripts

## Mais um exemplo

Vamos criar mais um código

	roscd meu_projeto/scripts
	touch le_scan.py
	chmod a+x le_scan.py

Vamos editar:

	subl le_scan.py

Digite o código abaixo:

```python
#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def scaneou(dado):
	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	print("Leituras:")
	print(np.array(dado.ranges).round(decimals=2))
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))

	


if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)



	while not rospy.is_shutdown():
		print("Oeee")
		velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))
		velocidade_saida.publish(velocidade)
		rospy.sleep(2)

```

Agora precisamos compilar novamente

    cd ~/catkin_ws/src
    catkin_make

Para conseguir executar, abra **um novo terminal** e digite:

    rosrun meu_projeto le_scan.py









