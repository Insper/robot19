package br.insper.robot19;

import java.awt.*;
import java.io.IOException;

public class GraficoMap {
    GridMap map;
    int width;
    int height;
    Canvas screen;
    //BufferedImage imagem;

    /**
     *
     * @param g Um objeto do itpo GridMap
     */
    public GraficoMap (GridMap g){
        this(g, 800, 600    );
    }

    /**
     *
     * @param g Um objeto do tipo GridMap
     * @param width A largura desejada para a janela
     * @param height A altura desejada para a janela
     */
    public GraficoMap(GridMap g, int width, int height){
        this.map = g;
        this.width = width;
        this.height = height;
        screen = new Canvas("Proj 3 Busca", this.width, this.height, Color.lightGray);

    }

    /**
     * Desenha uma representação gráfica do map que foi passado
     *
     * Vocês vão precisar alterar esta classe para mostrar nós expandidos e rota
     */
    public void desenha(){
        int h = map.getHeight();
        int w = map.getWidth();

        int sqx = this.width/w;
        int sqy = this.height/h;


        for (int i=0; i < h; i ++ ){
            for (int j = 0; j < w; j++){


                switch  (map.getBlockType(i, j)) {
                    case FREE:
                        screen.setForegroundColor(Color.WHITE);
                        break;
                    case WALL:
                        screen.setForegroundColor(Color.BLACK);
                        break;
                    case SAND:
                        screen.setForegroundColor(Color.YELLOW);
                        break;
                    case METAL:
                        screen.setForegroundColor(Color.BLUE);
                        break;
                }

                screen.fillRectangle(j*sqx, i*sqy, sqx-2, sqy-2);
            }
        }

        int[] goal = map.getGoal();
        int[] start = map.getStart();

        screen.setForegroundColor(Color.GREEN);
        screen.fillCircle(start[1]*sqx-sqx/2, start[0]*sqy-sqy/2, sqx/2);

        screen.setForegroundColor(Color.RED);
        screen.fillCircle(goal[1]*sqx-sqx/2, goal[0]*sqy-sqy/2, sqx/2);

    }

    /**
     * Saves a png file with what's shown in the Canvas
     */
    public void saveFile(String filename){
        screen.saveFile(filename);
    }

    /**
     *
     */
    public void setAndDrawSolucao(RobotAction[] solucao){
        if(solucao == null) {
            System.out.println("Nao foi encontrada solucao para o problema");
        } else {
            int[] s = map.getStart();
            Block atual = new Block(s[0], s[1], map.getBlockType(s[0], s[1]));
            System.out.print("Solução: ");
            for (RobotAction a : solucao) {
                System.out.print(", " + a);
                Block next = map.nextBlock(atual, a);
                map.setRoute(next.row, next.col);
                plotStep(atual, next);
                atual = next;
            }
        }

    }

    /**
     * Desenha uma linha entre dois blocos
     * @param atual
     * @param next
     */
    public void plotStep(Block atual, Block next){
        int h = map.getHeight();
        int w = map.getWidth();

        int sqx = this.width/w;
        int sqy = this.height/h;

        screen.setForegroundColor(Color.GREEN);
        screen.drawLine(atual.col*sqx + sqx/2, atual.row*sqy + sqy/2, next.col*sqx + sqx/2, next.row*sqy + sqy/2);

    }

    public static void main(String[] args) {
        GridMap map = null;
        try {
            map = GridMap.fromFile("map_teste.txt");
        } catch (IOException e) {
            e.printStackTrace();
        }
        /**
         * Situação: mostra o mapa inicial
         */
        GraficoMap grafico = new GraficoMap(map);
        grafico.desenha();
        grafico.saveFile("Busca1.png");

        /**
         * Agora vamos fazer uma busca em largura
         */

        // Bloco inicial
        int s[] = map.getStart();
        Block sb = new Block(s[0], s[1], map.getBlockType(s[0], s[1]));

        // Bloco final
        int f[] = map.getGoal();
        Block g = new Block(f[0], f[1], map.getBlockType(f[0], f[1]));

        // Rodando a busca
        BuscaLargura busca = new BuscaLargura(map, sb, g);
        RobotAction[] solucao = busca.resolver();

        // Requisitando o plot da solução sobre o gráfico
        grafico.setAndDrawSolucao(solucao);
        grafico.saveFile("Resolvido.png");


    }
}
