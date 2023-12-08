#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <queue>
#include <limits>
using namespace std;

vector<double> dijkstra(const vector<vector<pair<int, double>>>& graph, int start, vector<int>& parent);

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <filename>" << endl;
        return 1;
    }

    string filename = argv[1];
    ifstream inputFile(filename);

    if (!inputFile.is_open()) {
        cerr << "Couldn't open the file " << filename << endl;
        return 1;
    }

    int n, m;
    inputFile >> n >> m;

    vector<vector<pair<int, double>>> graph(n);

    for (int i = 0; i < m; ++i) {
        int u, v;
        double weight;
        inputFile >> u >> v >> weight;
        graph[u - 1].emplace_back(v - 1, weight);
        graph[v - 1].emplace_back(u - 1, weight);
    }

    // print graph
    for (int i = 0; i < n; ++i) {
        cout << "Node " << i + 1 << ": ";
        for (const auto& edge : graph[i]) {
            cout << "(" << edge.first + 1 << ", " << edge.second << ") ";
        }
        cout << endl;
    }
    cout << endl;

    vector<int> parent;

    // Example: Run Dijkstra's algorithm from node 0
    int startNode = 0;
    vector<double> shortestDistances = dijkstra(graph, startNode, parent);

    cout << "Shortest distances from node " << startNode + 1 << " to:\n";
    for (int i = 0; i < n; ++i) {
        cout << "Node " << i + 1 << ": " << shortestDistances[i] << '\n';
    }

    cout << "\nParent vector:\n";
    for (int i = 0; i < n; ++i) {
        cout << "Node " << i + 1 << " parent: " << parent[i] + 1 << '\n';
    }

    return 0;
}

vector<double> dijkstra(const vector<vector<pair<int, double>>>& graph, int start, vector<int>& parent) {
    int n = graph.size();
    vector<double> dist(n, numeric_limits<double>::max());
    dist[start] = 0;
    parent.assign(n, -1);

    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
    pq.push({0, start});

    while (!pq.empty()) {
        int currNodeWithMinDistFromSource = pq.top().second;
        double distFromSourceToCurrNode = pq.top().first;
        pq.pop();

        if (distFromSourceToCurrNode > dist[currNodeWithMinDistFromSource]) continue;

        for (const auto& neighbor : graph[currNodeWithMinDistFromSource]) {
            int currNeighbor = neighbor.first;
            double weight = neighbor.second;

            if (dist[currNodeWithMinDistFromSource] + weight < dist[currNeighbor]) {
                dist[currNeighbor] = dist[currNodeWithMinDistFromSource] + weight;
                parent[currNeighbor] = currNodeWithMinDistFromSource;
                pq.push({dist[currNeighbor], currNeighbor});
            }
        }
    }

    return dist;
}
