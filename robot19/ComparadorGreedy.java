package br.insper.robot19;

import java.util.Comparator;

public class ComparadorGreedy implements Comparator<Node> {
    @Override
    public int compare(Node node, Node node0) {
        if (node.getH() < node0.getH()){
            return -1;
        }
        if (node.getH() > node0.getH()){
            return 1;
        }
        else{return 0;}
    }
}