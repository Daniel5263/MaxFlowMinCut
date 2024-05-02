import java.util.*;

public class Main {
    static final int INF = Integer.MAX_VALUE;
    static final int V = 19;

    // Ford-Fulkerson algorithm to find maximal flow
    static int fordFulkerson(int[][] graph, int source, int sink, boolean[][] residualGraph) {
        int V = graph.length;
        int[][] residual = new int[V][V];
        for (int i = 0; i < V; i++)
            System.arraycopy(graph[i], 0, residual[i], 0, V);

        int[] parent = new int[V];
        int maxFlow = 0;

        while (bfs(residual, source, sink, parent)) {
            int pathFlow = INF;
            for (int v = sink; v != source; v = parent[v]) {
                int u = parent[v];
                pathFlow = Math.min(pathFlow, residual[u][v]);
            }

            for (int v = sink; v != source; v = parent[v]) {
                int u = parent[v];
                residual[u][v] -= pathFlow;
                residual[v][u] += pathFlow;
            }

            maxFlow += pathFlow;

            // Print augmenting path step by step
            System.out.println("Augmenting Path:");
            for (int v = sink; v != source; v = parent[v]) {
                int u = parent[v];
                System.out.println(v + " -> " + u);
            }
            System.out.println("Flow: " + pathFlow);
            System.out.println("Max Flow: " + maxFlow);
            System.out.println();
        }

        // Populate the residualGraph matrix
        for (int i = 0; i < V; i++) {
            for (int j = 0; j < V; j++) {
                residualGraph[i][j] = (graph[i][j] - residual[i][j] > 0);
            }
        }

        return maxFlow;
    }


    // Breadth-First Search to find augmenting path
    static boolean bfs(int[][] rGraph, int source, int sink, int[] parent) {
        int V = rGraph.length;
        boolean[] visited = new boolean[V];
        Arrays.fill(visited, false);

        Queue<Integer> queue = new LinkedList<>();
        queue.add(source);
        visited[source] = true;
        parent[source] = -1;

        while (!queue.isEmpty()) {
            int u = queue.poll();
            for (int v = 0; v < V; v++) {
                if (!visited[v] && rGraph[u][v] > 0) {
                    queue.add(v);
                    parent[v] = u;
                    visited[v] = true;
                }
            }
        }

        return visited[sink];
    }

    // DFS to find reachable vertices
    static void dfs(int[][] rGraph, int s, boolean[] visited) {
        visited[s] = true;
        for (int i = 0; i < rGraph.length; i++) {
            if (rGraph[s][i] > 0 && !visited[i]) {
                dfs(rGraph, i, visited);
            }
        }
    }

    // Find minimal cut using DFS
    static void findMinCut(int[][] graph, int s, int t) {
        // Create a residual graph and fill the residual graph with given capacities
        int[][] rGraph = new int[graph.length][graph.length];
        for (int i = 0; i < graph.length; i++) {
            System.arraycopy(graph[i], 0, rGraph[i], 0, graph.length);
        }

        // This array is filled by BFS and to store path
        int[] parent = new int[graph.length];

        // Augment the flow while there is path from source to sink
        while (bfs(rGraph, s, t, parent)) {
            // Find minimum residual capacity of the edges along the path
            int pathFlow = INF;
            for (int v = t; v != s; v = parent[v]) {
                int u = parent[v];
                pathFlow = Math.min(pathFlow, rGraph[u][v]);
            }

            // Update residual capacities of the edges and reverse edges along the path
            for (int v = t; v != s; v = parent[v]) {
                int u = parent[v];
                rGraph[u][v] -= pathFlow;
                rGraph[v][u] += pathFlow;
            }
        }
        // Flow is maximum now, find vertices reachable from s
        boolean[] isVisited = new boolean[graph.length];
        dfs(rGraph, s, isVisited);

        // Print all edges that are from a reachable vertex to non-reachable vertex
        System.out.println("Minimal Cut Edges:");
        for (int i = 0; i < graph.length; i++) {
            for (int j = 0; j < graph.length; j++) {
                if (graph[i][j] > 0 && isVisited[i] && !isVisited[j]) {
                    System.out.println(i + " - " + j);
                }
            }
        }
    }

    public static void main(String[] args) {
        int[][] graph = new int[V][V];

        graph[0][1] = 9;
        graph[0][2] = 9;
        graph[0][3] = 6;
        graph[0][4] = 9;
        graph[1][5] = 3;
        graph[1][2] = 8;
        graph[2][3] = 3;
        graph[2][5] = 2;
        graph[2][6] = 6;
        graph[2][7] = 8;
        graph[3][4] = 4;
        graph[3][7] = 6;
        graph[3][8] = 6;
        graph[4][8] = 12;
        graph[5][6] = 15;
        graph[6][7] = 1;
        graph[6][9] = 6;
        graph[7][10] = 8;
        graph[7][11] = 8;
        graph[8][11] = 3;
        graph[9][10] = 4;
        graph[9][12] = 6;
        graph[9][13] = 9;
        graph[10][11] = 6;
        graph[10][13] = 6;
        graph[10][14] = 5;
        graph[11][14] = 3;
        graph[11][15] = 9;
        graph[12][13] = 3;
        graph[12][18] = 18;
        graph[13][14] = 2;
        graph[13][17] = 12;
        graph[13][18] = 8;
        graph[14][15] = 9;
        graph[14][16] = 12;
        graph[14][17] = 6;
        graph[15][16] = 5;
        graph[16][17] = 5;
        graph[17][18] = 8;

        // Set capacities for reverse edges to make the graph undirected
        for (int i = 0; i < V; i++) {
            for (int j = 0; j < V; j++) {
                if (graph[i][j] > 0) {
                    graph[j][i] = graph[i][j];
                }
            }
        }

        Scanner scanner = new Scanner(System.in);
        System.out.print("Enter source vertex: ");
        int source = scanner.nextInt();
        System.out.print("Enter sink vertex: ");
        int sink = scanner.nextInt();

        // Find maximal flow
        boolean[][] residualGraph = new boolean[V][V];
        int maxFlow = fordFulkerson(graph, source, sink, residualGraph);
        System.out.println("Maximal Flow: " + maxFlow);

        // Find minimum cut
        findMinCut(graph, source, sink);

        System.out.println();
    }
}