
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

    /**
     * A queue that keeps track of the cost of inserted elements and builds a minimum spanning tree
     */
    private class SearchQueue {
        MappingComparator comparator=new MappingComparator();
        PriorityQueue<V> priorityQueue=new PriorityQueue<>(comparator);
        Map<V,V> precedingMap =new HashMap<>();// basis for the MST

        /**
         * A comparator with an internal map of ordinal numbers for the objects to be compared
         */
        private class MappingComparator implements Comparator<V> {
            Map<V,Double> costMap=new HashMap<>();
            Map<V,Double> heuristicCostMap=new HashMap<>();

            /**
             * Accesses the cost mapped to the specified key (actual+heuristic).
             * @param key The key who's cost should be accessed
             * @return The stored cost, or infinity if there isn't one
             */
            double cost(V key) {
                Double cost=costMap.get(key);
                Double heuristicCost=heuristicCostMap.get(key);
                if (cost!=null) return cost+heuristicCost;// cost and heuristicCost can't be assigned separately
                else return Double.POSITIVE_INFINITY;
            }

            /**
             * Accesses the actual cost mapped to a specified key.
             * @param key The key who's actual cost to access
             * @return The stored cost, or infinity if there isn't one
             */
            double actualCost(V key) {
                Double cost=costMap.get(key);
                if (cost!=null) return cost;
                else return Double.POSITIVE_INFINITY;
            }

            /**
             * Maps the specified key to the specified costs,
             * but only if the specified costs are have a lower sum than the ones already present,
             * or the specified key does not have an associated cost.
             * @param key The key
             * @param cost The new cost
             * @param heuristicCost The new heuristic cost
             * @return true iff the cost was updated
             */
            boolean offer(V key, double cost, double heuristicCost) {
                Double currentCost=costMap.get(key);
                Double currentHeuristicCost=heuristicCostMap.get(key);
                if (currentCost==null || (cost+heuristicCost)<(currentCost+currentHeuristicCost)) {
                    costMap.put(key,cost);
                    heuristicCostMap.put(key,heuristicCost);
                    return true;
                }
                return false;
            }

            @Override
            public int compare(V o1, V o2) {
                return Double.compare(cost(o1),cost(o2));
            }
        }

        /**
         * Attempts to insert an element into the queue with a specified cost divided into actual and heuristic,
         * and into the MST at a specified point of attachment.
         * @param element The element to be inserted
         * @param cost The actual cost to insert the element with
         * @param heuristicCost The heuristicCost to insert the element with
         * @param tail The element to consider preceding in the spanning tree
         */
        void offer(V element, double cost, double heuristicCost, V tail) {
            if (!comparator.offer(element,cost,heuristicCost)) return;
            assert precedingMap.containsKey(tail) || tail==null;
            precedingMap.put(element,tail);
            priorityQueue.remove(element);
            priorityQueue.offer(element);
        }
        /**
         * Attempts to insert an element into the queue with a specified cost
         * and into the MST at a specified point of attachment.
         * @param element The element to be inserted
         * @param cost The actual cost to insert the element with
         * @param tail The element to consider preceding in the spanning tree
         */
        void offer(V element, double cost, V tail) {
            offer(element, cost, 0, tail);
        }

        /**
         * Remove and return the smallest element currently in the queue
         * @return The smallest element in the queue
         */
        V poll() { return priorityQueue.poll(); }

        /**
         * The cost the queue associates with the specified element.
         * @param element The element who's cost to check
         * @return The stored cost, or infinity if there is none
         */
        double cost(V element) { return comparator.actualCost(element); }

        /**
         * Generates an ordered list of the path stored in the MST from a root node to the specified element
         * @param element The element to which a path is to be generated
         * @return The generated path to the specified element
         */
        List<V> path(V element) {
            List<V> reversePath=new ArrayList<>();
            V preceding=element;
            do {
                reversePath.add(preceding);
            } while ((preceding=precedingMap.get(preceding))!=null);
            return reverse(reversePath);
        }

        @SuppressWarnings("unchecked")
        private List<V> reverse(List<V> reverse) {
            Object[] temp=new Object[reverse.size()];
            int i=reverse.size();
            for (V node:reverse) {
                temp[--i]=node;
            }
            List<V> list=new ArrayList<>(temp.length);
            for (Object o:temp) {
                list.add((V)o);
            }
            return list;
        }
    }

    public Result<V> searchDijkstra(V start, V goal) {
        int visitedNodes = 0;
        SearchQueue queue=new SearchQueue();
        queue.offer(start,0,null);
        Set<V> visited=new HashSet<>();
        V current;
        while ((current=queue.poll())!=null) {
            visitedNodes++;
            visited.add(current);
            if (current.equals(goal)) {
                return new Result<>(true,start,current,queue.cost(current),queue.path(current),visitedNodes);
            }
            List<DirectedEdge<V>> outgoing=graph.outgoingEdges(current);
            for (DirectedEdge<V> edge:outgoing) {
                V node=edge.to();
                if (!visited.contains(node)) queue.offer(node,edge.weight()+queue.cost(current),current);
            }
        }
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
