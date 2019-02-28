# OpenCV e Jupyter com suporte a SIFT e DNN no Linux 18.04

[Fonte de parte deste tutorial](https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/)

Problema:  versão *default* do OpenCV no Ubuntu 18.04 não tem SIFT nem suporte a redes neurais, necessários para usar *YOLO* ou *MobileNet*. T

Este tutorial foi elaborado com ajuda do aluno **Bruno Domingues** (Robótica - 2019)

**Nota:** [aqui](./instalar_opencv_jupyter.sh) estão todos os comandos em sequências. Você pode só rodar o *script* em vez de seguir o tutorial


    sudo apt-get install build-essential cmake unzip pkg-config


    sudo apt-get install libjpeg-dev libpng-dev libtiff-dev




    sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev

    sudo apt-get install libxvidcore-dev libx264-dev


    sudo apt-get install libgtk-3-dev

    sudo apt-get install libatlas-base-dev gfortran


    sudo apt-get install python3-dev

    sudo apt install python3-numpy


    mkdir ~/tmp
    cd ~/tmp

Baixe os arquivos a seguir do Github, ou veja se há alguma cópia local disponibilizada pelos professores

    wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.4.zip
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.4.zip

Descompactar os arquivos:

    unzip opencv.zip

    unzip opencv_contrib.zip

Renomear as pastas:

    mv opencv-3.4.4/ opencv
    mv opencv_contrib-3.4.4/ opencv_contrib

Agora vamos usar o cmake para *compilar*  a OpenCV:


    cd ~/tmp/opencv
    mkdir build
    cd build
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D INSTALL_PYTHON_EXAMPLES=ON \
        -D INSTALL_C_EXAMPLES=OFF \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D OPENCV_EXTRA_MODULES_PATH=~/tmp/opencv_contrib/modules \
        -D PYTHON_EXECUTABLE=/usr/bin/python3 \
        -D BUILD_EXAMPLES=ON ..

É importante que as linhas finais não digam que aconteceram erros

Agora vamos compilar (deve levar vários minutos):

    make -j4

O comando `make` apenas cria os binários compilados. Para copiá-los para seus devidos lugares, precisamos fazer:

    sudo make install

E depois:

    sudo ldconfig

Agora **abra um novo terminal** (`Ctrl` `Alt` `T`) e digite o comando a seguir para verificar a instalação:

    pkg-config --modversion opencv

Deve aparecer a versão da OpenCV que você está usando (no caso deste guia, deve ser a 3.4.4)


## PYTHONPATH - precisamos configurar o Python para enxergar a OpenCV que acabamos de instalar

    echo "export PYTHONPATH=/usr/lib/python:$PYTHONPATH" >> ~/.bashrc


## Jupyter

Para instalar o Jupyter, por favor faça:

    sudo apt install python3-pip

Depois:

    sudo -H pip3 install jupyter

**nota** no Ubuntu é importante chamar explicitamente a versão $3$ das ferramentas Python: `python3`, `pip3`, etc. Isto acontece porque o Python $2$ continua presente e é o `python` default.


    python3 -m ipykernel install --user



