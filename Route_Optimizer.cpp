#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include<bits/stdc++.h>
using namespace std;

// Dijkstra's Algorithm
vector<int> dijkstra(const vector<vector<int>> &graph, int start) {
    int n = graph.size();
    vector<int> dist(n, INT_MAX);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        int u = pq.top().second;
        int d = pq.top().first;
        pq.pop();

        if (d > dist[u]) continue;

        for (int v = 0; v < n; ++v) {
            if (graph[u][v] != 0) { // If there is an edge
                int new_dist = dist[u] + graph[u][v];
                if (new_dist < dist[v]) {
                    dist[v] = new_dist;
                    pq.push({dist[v], v});
                }
            }
        }
    }

    return dist;
}

// A* Algorithm
vector<int> astar(const vector<vector<int>> &graph, const vector<int> &heuristics, int start) {
    int n = graph.size();
    vector<int> dist(n, INT_MAX);
    vector<int> f(n, INT_MAX);
    vector<bool> visited(n, false);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    dist[start] = 0;
    f[start] = heuristics[start];
    pq.push({f[start], start});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (visited[u]) continue;
        visited[u] = true;

        for (int v = 0; v < n; ++v) {
            if (graph[u][v] != 0 && !visited[v]) {
                int new_dist = dist[u] + graph[u][v];
                if (new_dist < dist[v]) {
                    dist[v] = new_dist;
                    f[v] = dist[v] + heuristics[v];
                    pq.push({f[v], v});
                }
            }
        }
    }

    return dist;
}

// Dynamic Programming for TSP
int tsp(int mask, int pos, const vector<vector<int>> &dist, vector<vector<int>> &dp) {
    int n = dist.size();
    if (mask == (1 << n) - 1) {
        return dist[pos][0]; // Return to the starting point
    }

    if (dp[mask][pos] != -1) {
        return dp[mask][pos];
    }

    int ans = INT_MAX;

    for (int city = 0; city < n; ++city) {
        if ((mask & (1 << city)) == 0) {
            int newAns = dist[pos][city] + tsp(mask | (1 << city), city, dist, dp);
            ans = min(ans, newAns);
        }
    }

    return dp[mask][pos] = ans;
}

int main() {
    // Sample input for Dijkstra's and A* algorithms
    vector<vector<int>> graph = {
        {0, 10, 15, 20},
        {10, 0, 35, 25},
        {15, 35, 0, 30},
        {20, 25, 30, 0}
    };

    vector<int> heuristics = {10, 5, 2, 1}; // Sample heuristics for A*

    // Dijkstra's Algorithm
    cout << "Dijkstra's Algorithm:" << endl;
    vector<int> dist_dijkstra = dijkstra(graph, 0);
    for (int i = 0; i < dist_dijkstra.size(); ++i) {
        cout << "Distance from node 0 to node " << i << " is " << dist_dijkstra[i] << endl;
    }

    // A* Algorithm
    cout << "\nA* Algorithm:" << endl;
    vector<int> dist_astar = astar(graph, heuristics, 0);
    for (int i = 0; i < dist_astar.size(); ++i) {
        cout << "Distance from node 0 to node " << i << " is " << dist_astar[i] << endl;
    }

    // TSP Problem
    cout << "\nDynamic Programming for TSP:" << endl;
    vector<vector<int>> tsp_dist = {
        {0, 10, 15, 20},
        {10, 0, 35, 25},
        {15, 35, 0, 30},
        {20, 25, 30, 0}
    };
    int n = tsp_dist.size();
    vector<vector<int>> dp(1 << n, vector<int>(n, -1));
    int result = tsp(1, 0, tsp_dist, dp);
    cout << "The shortest route cost is: " << result << endl;

    return 0;
}
