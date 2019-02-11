# Instalando OpenCV no IPython notebook

Precisamos usar Python 2 este é o Python mais suportado e testado na versão do ROS que usaremos. Assim fica mais fácil aproveitar código feito agora no robô.


Abra um terminal, e veja se já não tem um ambiente com Python 2.

    conda env list

Para saber qual é o ambiente ativo atualmente dê o comando:

    conda info --envs

A saída deverá ser algo assim:

    # conda environments:
    #
    Python27                 //anaconda/envs/Python27
    _build                   //anaconda/envs/_build
    root                  *  //anaconda

O asterisco marca o ambiente que está sendo usando

Para ativar outro ambiente:

**No Windows**
    activate <font color=red>NOME_DO_AMBIENTE</font>

**No Linux ou Mac**
    source activate <font colore=red>NOME_DO_AMBIENTE</font>


Para criar um environment Python 2.7 faça:

    conda create -n python2amb python=2.7 anaconda

Depois, para mudar para o ambiente Python 2.7, fa;ca

**No Windows**
    activate <font color=red>python2amb</font>

**No Linux ou Mac**
    source activate <font colore=red>python2amb</font>

No que ao invés de **python2amb** você pode usar o nome que quiser.

Para instalar a OpenCV

    conda install opencv

## Caso tenha problemas

### No Windows ou no OSX

Faça:

    conda install -c conda-forge opencv


**Alternativa para Windows:**


Baixe a OpenCV do link [https://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.11/](https://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.11/) e a instale em C:\OpenCV

Depois copie o conteúdo de C:\OpenCV\build\python\x86\2.7\ e cole na pasta 
**Pasta_Anaconda**/envs/**NOME_DO_SEU_AMBIENTE**/lib/python2.7/site-packages

No lugar de **Pasta_Anaconda** inclue a pasta em que seu Anaconda está instalado. Por exemplo C:\anaconda

No lugar de **NOME_DO_SEU_AMBIENTE** coloque **python2amb** ou o nome que deu ao seu ambiente Python
