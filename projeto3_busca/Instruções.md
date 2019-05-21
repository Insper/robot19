# Projeto de Busca Greedy e A*

## Objetivo
O objetivo deste projeto é implementar as diversas modalidades de busca heurística aplicadas ao planejamento de 
caminho em robótica, e a sua integração com o simulador VREP.  

## Implementação da busca

O projeto Java BuscaVREP faz a implementação da busca em largura (busca geral do livro do Russel) em um mapa retangular reticulado previamente fornecido no formato de texto. Os símbolos utilizado são:
- **#** - parede (intransponível)
- **0** : posição inicial
- **G** : posição alvo (goal)
- **.** : célula livre (custo=1)
- `*` (asterisco) : rota encontrada
- **3** : suerfície de metal (custo=3)
- **X** : superfície de areia (custo=10)

Para executar o projeto sem a integração com o simulador, basta executar a classe `ExecutaBusca`.
 
## Dicas para a implementação da busca

O projeto Java fornecido executa a **busca em largura** em uma mapa de grade.
Uma vez que ela faz parte da classe de buscas cegas, ou seja, que não usam
informações específicas sobre o problema, implementar as buscas informadas
(como a **buca gulosa** e a **busca A* **) requer:

1. Acrescentar a informação do valor da função heurística $h(n)$ no nó de busca
2. Criar um mecanismo para sempre ordenar a fronteira em ordem do valor desejado
(função heurística  $h(n)$ ou função de avaliação $f(n)=g(n)+h(n)$). Esse mecanismo
já está implementado em Java através da classe `PriorityQueue` (ver [este link](https://stackoverflow.com/questions/683041/how-do-i-use-a-priorityqueue))

Além disso, implementar o algoritmo da busca em grafo (aquela que evita estados repetidos) requer também armazenar uma lista onde cada estado aparece apenas uma vez.
A classe que implementa o estado do problema é
a classe `Block` que já possui os métodos necessários para ser armazenada em conjuntos de exemplares únicos (como [HashSet](https://docs.oracle.com/javase/7/docs/api/java/util/HashSet.html)).

Os arquivos fornecidos a serem manipulados são:
 - `mapa_teste.txt`: exemplo de mapa que o projeto consegue carregar
 - `BuscaLargura.java`: exemplo de implementação da busca em largura (busca geral do livro do Russel), que deve ser modificado para implementar as demais modalidades de busca.
 - `Node.java`: que deve ser atualizada para armazenar também o valor da função heurístcia como seu propriedade
 - `ExecutaBusca.java`: exemplo de chamada à busca em largura, que deverá invocar os demais tipos de busca conforme o caso, além de invocar a API do simulador V-REP, quando a integração for realizada
 - `GraficoMap.java`: exemplo de chamada à busca em largura que explica como usar a classe `GraficoMap` que faz o *output* gráfico como [na figura](./java/Resolvido.png)

 
 ## Integração com o simulador VREP
 
 O pacote `br.insper.robot19.vrep` contém as classes necessárias à integração com o simulador VREP.
 
 O arquivo de execução da busca com integração ao VREP deve fazer com que o mapa seja lido diretamente do simulador,
 a partir da classe VREPWorld, e posteriormente o robô deverá executar comando a comando encontrado através da 
 classe VREPRobot. A criação dessas classes deve ser realizada da seguinte forma:
 
```java
VrepSimulator sim = VrepSimulator.getInstance();
VrepWorld world = sim.createWorld();
VrepRobot robot = sim.createRobot();
```

A integração deve ser feita com a leitura dos métodos disponíveis nas classes (gerar Javadoc)
    
### Precondições

As classes de  pressupõe que:
 1. O simulador está aberto e rodando a simulação em modo Real Time (símbolo do relógio acionado - figura 1)
 2. O simulador escuta comandos de um cliente externo na porta 19997 (comportamento padrão)
 3. A cena sendo simulada seja baseada em um ResizableFloor (5 a 25 m)
 4. A cena a contém um e somente um robô do tipo Pioneer P3DX, e uma porta de acionamento automático
 (sliding door), usada como posição alvo do robô
 4. Os obstáculos são construídos a partir de objetos do tipo Wall, preferencialmente com altura de com atura de 240cm
 5. A superfície de metal é simulada com o elemento "5m X 5m Metallic Floor"
 6. A superfície de areia é simulada com o elemento "Bump", cuja altura deve ser bem pequena para que o robô consiga pasar por cima
 
 O tipo de chão padrão ao se iniciar o V-REP é o Resizable Floor (5 to 25m), que já está correto.
 Os demais elementos do cenário só serão reconhecidos na geração do mapa de grade se estiverem dentro das opções descritas acima
 
 
 ![Tela do simulador V-REP](VREP1.PNG "Figura 1: tela do V-REP")
 **Figura 1:** Tela do V-REP com destaque para o botão Real Time (círculo vermelho), Play (círculo verde) e para o robô que deve se usado na simulação
 

 ### Rubrica

 A [rubrica do projeto está disponível](RUBRICA_proj3.pdf)
 
