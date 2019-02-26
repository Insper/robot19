
# Entregável 3 de Visão Computacional e Robótica


Entrega até 12/03 ao fim do atendimento

Pode ser feito **em duplas**

Nesta semana vamos trabalhar com um assunto extremamente atual: reconhecimento de objetos e rastreamento


Referências:

[https://www.pyimagesearch.com/2018/07/30/opencv-object-tracking/](https://www.pyimagesearch.com/2018/07/30/opencv-object-tracking/)

[https://github.com/iArunava/YOLOv3-Object-Detection-with-OpenCV/](https://github.com/iArunava/YOLOv3-Object-Detection-with-OpenCV/)

[https://www.pyimagesearch.com/2017/09/11/object-detection-with-deep-learning-and-opencv/](https://www.pyimagesearch.com/2017/09/11/object-detection-with-deep-learning-and-opencv/)

Ouça a explicacão do professor sobre rastreamento e deteção

### 1. Executar os três exemplos

Há três exemplos: `mobilenet_detection`, `yolov3_detection` e `tracking`.

Os dois primeiros são reconhecedores de objetos, e o último é de rastreamento


Um dos arquivos abaixo precisa ser baixado e salvo nas pasta  `yolov3_detection/yolov3-coco` .

[https://www.dropbox.com/s/013ogt2bhwfzxwb/yolov3.weights?dl=0](https://www.dropbox.com/s/013ogt2bhwfzxwb/yolov3.weights?dl=0) ou [https://pjreddie.com/media/files/yolov3.weights](https://pjreddie.com/media/files/yolov3.weights)

## 2. Identificar objeto

Escolha um dos projetos: `yolov3_detection` ou `mobilenet_detection` para basear seu código. 

Neste projeto, escolha uma categoria de objetos que o reconhecedor reconhece. Diga aqui qual foi sua escolha

## 3. Rastrear

Implemente a seguinte funcionalidade: sempre que o objeto identificado em (2) estiver presente por mais que 5 frames seguidos, pare de rodar a identificação e comece a rastreá-lo.

Faça um vídeo com a  demonstração funcionalidade e mostre o link ao professor

## 4. Simulador

 Rode o simulador do Turtlebot (use o Waffle).  Veja o guia em [../guides/simulador_ros.md](../guides/simulador_ros.md)
 
 Documente aqui as linhas necessárias para teleop e para abrir o Rviz
 
 Faça um screenshot do seu simulação em execução

## 5. Robô quadrado

Faça [este tutorial](../guides/projeto_rospython.md) de como criar um projeto Python que comanda o robô simulado.

Usando o simulador, crie um código que faça o robô fazer uma trajetória que aproxima um quadrado.

Baseie-se no código `roda.py`, construído durante o tutorial 

## 6. Robô indeciso

Usando o simulador e o LIDAR simulado, faça um robô avançar quando o obstáculo bem à sua frente estiver a menos de 1.0m e recuar quando estiver a mais de 1.02 m.

Baseie-se no código `le_scan.py` e `roda.py`, desenvolvidos [durante o tutorial](../guides/projeto_rospython.md)
