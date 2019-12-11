
import java.util.*;

import java.util.stream.Collectors;


public class PathFinder<V> {

    private DirectedGraph<V> graph;
    private long startTimeMillis;


    public PathFinder(DirectedGraph<V> graph) {
        this.graph = graph;
    }


    public class Result<V> {
        public final boolean success;
        public final V start;
        public final V goal;
        public final double cost;
        public final List<V> path;
        public final int visitedNodes;
        public final double elapsedTime;

        public Result(boolean success, V start, V goal, double cost, List<V> path, int visitedNodes) {
            this.success = success;
            this.start = start;
            this.goal = goal;
            this.cost = cost;
            this.path = path;
            this.visitedNodes = visitedNodes;
            this.elapsedTime = (System.currentTimeMillis() - startTimeMillis) / 1000.0;
        }

        public String toString() {
            String s = "";
            s += String.format("Visited nodes: %d\n", visitedNodes);
            s += String.format("Elapsed time: %.1f seconds\n", elapsedTime);
            if (success) {
                s += String.format("Total cost from %s -> %s: %s\n", start, goal, cost);
                s += "Path: " + path.stream().map(x -> x.toString()).collect(Collectors.joining(" -> "));
            } else {
                s += String.format("No path found from %s", start);
            }
            return s;
        }
    }


    public Result<V> search(String algorithm, V start, V goal) {
        startTimeMillis = System.currentTimeMillis();
        switch (algorithm) {
        case "random":   return searchRandom(start, goal);
        case "dijkstra": return searchDijkstra(start, goal);
        case "astar":    return searchAstar(start, goal);
        }
        throw new IllegalArgumentException("Unknown search algorithm: " + algorithm);
    }


    public Result<V> searchRandom(V start, V goal) {
        int visitedNodes = 0;
        LinkedList<V> path = new LinkedList<>();
        double cost = 0.0;
        Random random = new Random();

        V current = start;
        path.add(current);
        while (current != null) {
            visitedNodes++;
            if (current.equals(goal)) {
                return new Result<>(true, start, current, cost, path, visitedNodes);
            }

            List<DirectedEdge<V>> neighbours = graph.outgoingEdges(start);
            if (neighbours == null || neighbours.size() == 0) {
                break;
            } else {
                DirectedEdge<V> edge = neighbours.get(random.nextInt(neighbours.size()));
                cost += edge.weight();
                current = edge.to();
                path.add(current);
            }
        }
        return new Result<>(false, start, null, -1, null, visitedNodes);
    }

    private class SearchQueue {
        MappingComparator comparator=new MappingComparator();
        PriorityQueue<V> priorityQueue=new PriorityQueue<>(comparator);
        Map<V,V> precedingMap =new HashMap<>();

        private class MappingComparator implements Comparator<V> {
            Map<V,Double> costMap=new HashMap<>();

            boolean contains(V key) {
                return costMap.containsKey(key);
            }

            double get(V key) { return costMap.get(key); }

            boolean put(V key, double cost) {
                Double currentCost=costMap.get(key);
                if (currentCost!=null && currentCost>cost) {
                    costMap.put(key,cost);
                    return true;
                }
                return false;
            }

            void remove (V key) {
                costMap.remove(key);
            }

            @Override
            public int compare(V o1, V o2) {
                assert o1!=null && o2!=null;
                return Double.compare(costMap.get(o1),costMap.get(o2));
            }
        }

        void offer(V element, double cost, V tail) {
            if (comparator.contains(element)) {
                priorityQueue.remove(element);
            }
            if (comparator.put(element,cost)) {
                precedingMap.put(tail,element);
            }
            priorityQueue.offer(element);
        }

        V poll() {
            V smallest=priorityQueue.poll();
            comparator.remove(smallest);
            return smallest;
        }

        double cost(V element) {
            return comparator.get(element);
        }

        List<V> path(V element) {
            List<V> reversePath=new ArrayList<>();
            V preceding=element;
            do {
                reversePath.add(preceding);
            } while ((preceding=precedingMap.get(preceding))!=null);
            List<V> path=new ArrayList<>(reversePath.size());
            int i=reversePath.size();
            for (V node:reversePath) {
                path.set(--i,node);
            }
            return path;
        }
    }

    // public Result(boolean success, V start, V goal, double cost, List<V> path, int visitedNodes) {

    public Result<V> searchDijkstra(V start, V goal) {
        int visitedNodes = 0;
        SearchQueue queue=new SearchQueue();
        queue.offer(start,0,null);
        V current;
        while ((current=queue.poll())!=null) {
            visitedNodes++;
            if (current.equals(goal)) {
                return new Result<>(true,start,current,queue.cost(current),queue.path(current),visitedNodes);
            }
            List<DirectedEdge<V>> outgoing=graph.outgoingEdges(current);
            for (DirectedEdge<V> edge:outgoing) {
                queue.offer(edge.to(),edge.weight()+queue.cost(current),current);
            }
        }

        /********************
         * TODO: Task 1
         ********************/
        return new Result<>(false, start, null, -1, null, visitedNodes);
    }
    

    public Result<V> searchAstar(V start, V goal) {
        int visitedNodes = 0;
        /********************
         * TODO: Task 3
         ********************/
        return new Result<>(false, start, null, -1, null, visitedNodes);
    }

}
