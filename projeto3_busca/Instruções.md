# Atividade de Busca A*

## Objetivo
O objetivo deste projeto é implementar as diversas modalidades de busca heurística aplicadas ao planejamento de 
caminho em robótica, e a sua integração com o simulador VREP.  

## Implementação da busca

O projeto Java BuscaVREP faz a implementação da busca em largura (busca geral do livro do Russel) em um mapa
retangular reticulado previamente fornecido no formato de texto. Os símbolos utilizads são:
- **#** - parede (intransponível)
- **0** : posição inicial
- **G** : posição alvo (goal)
- **.** : célula livre (custo=1)
- `*` (asterisco) : rota encontrada
- **3** : suerfície de metal (custo=3)
- **X** : superfície de areia (custo=10)

Os arquivos fornecidos a serem manipulados são:
 - `mapa_teste.txt`: exemplo de mapa que o projeto consegue carregar
 - `BuscaLargura.java`: exemplo de implementação da busca em largura (busca geral do livro do Russel),
 que deve ser modificado para implementar as demais modalidades de busca.
 - `ExecutaBusca.java`: exemplo de chamada à busca em largura, que deverá invocar os demais tipos de busca conforme o caso
 
 Para executar o projeto sem a integração com o simulador, basta executar a classe `ExecutaBusca`.
 
 
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
 1. O simulador está aberto e rodando a simulação em modo Real Time (símbolo do relógio acionado)
 2. O simulador escuta comandos de um cliente externo na porta 19997 (comportamento padrão)
 3. A cena sendo simulad seja baseada em um ResizableFloor (5 a 25 m)
 4. A cena a contém um e somente um robô do tipo Pioneer P3DX, e uma porta de acionamento automático
 (sliding door), usada como posição alvo do robô
 4. Os obstáculos são construídos a partir de objetos do tipo Wall, preferencialmente com altura de com atura de 240cm
 5. A superfície de metal é simulada com o elemento "5m X 5m Metallic Floor"
 6. A superfície de areia é simulada com o elemento "Bump", cuja altura deve ser bem pequena para que o robô consiga pasar por cima
 
 
