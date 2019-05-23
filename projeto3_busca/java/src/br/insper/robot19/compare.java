package br.insper.robot19;

import java.util.Comparator;

class HeuristicComparator implements Comparator<Node> {
    @Override
    public int compare(Node x, Node y)
    {
        // Assume neither string is null. Real code should
        // probably be more robust
        // You could also just return x.length() - y.length(),
        // which would be more efficient.
        if (x.getHeuristica() < y.getHeuristica())
        {
            return -1;
        }
        if (x.getHeuristica() > y.getHeuristica())
        {
            return 1;
        }
        return 0;
    }
}