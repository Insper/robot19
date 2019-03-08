
# Conserto do LD_LIBRARY_PATH no .bashrc


A variável de ambiente `$LD_LIBRARY_PATH` determina onde os sistemas *Unix* procuram por bibliotecas dinâmicas (arquivos com a extensão `*.so`) para executar. A versão em Linux do SSD tem um conflito entre os `LD_LIBRARY_PATH` do *ROS* e do *Altera*.




## Como consertar. 


Apague as duas linhas abaixo no seu arquivo `.bashrc`.

Para abrir o .bashrc use o *Sublime* (comando `subl`) ou o `gedit` . Por exemplo:

    subl ~/.bashrc

Apague a linha a seguir, que deve estar perto da `l.126`:

    export LD_LIBRARY_PATH=/home/borg/intelFPGA_lite/17.1/modelsim_ase/lib32



Ao redor da linha 138, apague:
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/melodic/lib/parrot_arsdk/


Insira no arquivo a linha a seguir:

    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/melodic/lib/parrot_arsdk/:/home/borg/intelFPGA_lite/17.1/modelsim_ase/lib32

Salve seu arquivo `.bashrc`. As alterações que você fez passarão a valer assim que abrir um **novo terminal**





