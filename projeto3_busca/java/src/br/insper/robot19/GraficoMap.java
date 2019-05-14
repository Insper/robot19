package br.insper.robot19;

import java.awt.*;
import java.io.IOException;

public class GraficoMap {
    GridMap map;
    int width;
    int height;
    Canvas screen;
    //BufferedImage imagem;

    public GraficoMap (GridMap g){
        this(g, 800, 600    );
    }

    public GraficoMap(GridMap g, int width, int height){
        this.map = g;
        this.width = width;
        this.height = height;
        screen = new Canvas("Proj 3 Busca", this.width, this.height, Color.white);

    }

    public void desenha(){
        int h = map.getHeight();
        int w = map.getWidth();

        int sqx = width/w;
        int sqy = height/h;


        for (int i=0; i < w; i ++ ){
            for (int j = 0; j < h; j++){
                screen.fillRectangle(i*sqx, j*sqy, sqx-2, sqy-2);
            }
        }

    }

    public static void main(String[] args) {
        GridMap map = null;
        try {
            map = GridMap.fromFile("map_teste.txt");
        } catch (IOException e) {
            e.printStackTrace();
        }
        GraficoMap grafico = new GraficoMap(map);
        grafico.desenha();
        grafico.saveFile("Busca1.png");

    }
}
