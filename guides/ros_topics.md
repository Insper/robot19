# Explorando tópicos e mensagens

## Como descobrir a lista de tópicos do ROS

O comando `rostopic list` mostra a lista de tópicos (*inputs* e *outputs*) disponiveis no ROS

Exemplo:

```bash 
borg@ubuntu:~$ rostopic list
/battery_state
/bumper
/cmd_vel
/cmd_vel_rc100
/diagnostics
/firmware_version
/imu
/joint_states
/magnetic_field
/motor_power
/odom
/reset
/rosout
/rosout_agg
/rpms
/scan
/sensor_state
/sound
/tf
/tf_static
/version_info
```


## Para inspecionar um tópico

O comando `rostopic echo TOPICO` permite ver os dados que estão circulando em algum dos tópicos. Por exemplo:

```bash 
borg@ubuntu:~$ rostopic echo /scan
header: 
  seq: 6426
  stamp: 
    secs: 1552313313
    nsecs: 600081469
  frame_id: "base_scan"
angle_min: 0.0
angle_max: 6.28318548203
angle_increment: 0.0174532923847
time_increment: 2.98899994959e-05
scan_time: 0.0
range_min: 0.119999997318
range_max: 3.5
ranges: [0.0, 0.0, 1.5889999866485596, 1.3539999723434448, 1.3240000009536743, 1.2130000591278076, 1.0499999523162842, 1.0889999866485596, 0.9599999785423279, 0.9599999785423279, 0.9039999842643738, 0.8640000224113464, 0.8450000286102295, 0.8069999814033508, 0.734000027179718, 0.7279999852180481
```

O final foi omitido porque a saída do `/scan` contém as leituras do scan laser, e são muitos dados.

### Descobrindo o tipo de dados de um tópico

Precisamos descobrir o **tipo** de dado de um certo tópico.

Um tópico é como se fosse um **objeto** e o **tipo** é *Classe*.

Para descobrir o tipo de dado de um tópico, usamos o comando `rostopic info TOPICO`.

Por exemplo:

```bash
borg@ubuntu:~$ rostopic info /scan
Type: sensor_msgs/LaserScan

Publishers: 
 * /turtlebot3_lds (http://192.168.0.150:40245/)

Subscribers: 
 * /turtlebot3_diagnostics (http://192.168.0.150:44731/)

```

Para descobrir quais as variáveis internas do dado do tópico, usamos o comando `rosmsg`:

```bash
borg@ubuntu:~$ rosmsg info sensor_msgs/LaserScan
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```

## Para escrever em um tópico 

Existem tópicos de leitura e tópicos de escrita. Por exemplo, é comum que o `cmd_vel` seja a velocidade desejada de um robô no ROS.

Podemos escrever a velocidade desejada usando o `rostopic pub` para o `cmd_vel`

O comando abaixo manda o robô ir para a frente com uma taxa de 10 vezes por segundo ( `-r 10` )

    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'




