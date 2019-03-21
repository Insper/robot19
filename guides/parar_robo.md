# Parar o robô

Para parar o robô, copie e cole o seguinte comando no terminal:

    rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
