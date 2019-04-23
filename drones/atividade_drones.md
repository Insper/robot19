# Drones

Esta atividade conta como opcional em sua nota de atividades de grupo. Pode ser mostrada até 30/04. 

## Execução

Se você usa a imagem do Ubuntu 18.04 fornecida pela disciplina, use o [guia de execução](execucao.md)

## Instalação

Caso seja uma instalação sua do Linux, vá para o [guia de instalação](instalacao.md)

## Simulador

Sobre [como executar o simulador de drones Bebop (Ubuntu 16.04 somente)](https://github.com/Insper/bebop_sphinx/blob/master/docs/instrucoes_sphinx.md) . Peça aos técnicos ou professor a máquina virtual do VirtualBox que já tem o simulador funcionando.

[Vídeo com instruções do simulador - parte 1](https://www.youtube.com/watch?v=VlviiwyvSu4)
[Vídeo com instruções do simulador - parte 2](https://www.youtube.com/watch?v=gfeORCX7F0w)

## Informações de segurança

Todos os que estiverem na quadra junto com o *drone* precisam usar equipamento de segurança:
* Óculos de proteção
* Sapato fechado e antiderrapante
* Calça comprida
* Vestimenta que cubra os braços

Além disso **ninguém deve tocar um drone em operação**

## Precauções adicionais

Você pode fazer a decolagem via código, mas faça sempre o pouso via terminal

Para fazer o pouso

    rostopic pub --once bebop/land std_msgs/Empty



