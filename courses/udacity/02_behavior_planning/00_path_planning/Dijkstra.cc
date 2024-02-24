#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <vector>
#include <limits>
#include <utility>
#include <stack>

using namespace std;

using Graph = map<string, vector<pair<string, int>>>;

class CompareDist {
public:
    bool operator()(pair<int, string> n1, pair<int, string> n2) {
        return n1.first > n2.first;
    }
};

void printPath(const map<string, string>& predecessors, const string& target) {
    stack<string> path;
    string at = target;
    while (predecessors.find(at) != predecessors.end()) {
        path.push(at);
        at = predecessors.at(at);
    }
    path.push(at); // Push the source node

    cout << "Shortest path: ";
    while (!path.empty()) {
        cout << path.top();
        path.pop();
        if (!path.empty()) cout << " -> ";
    }
    cout << endl;
}

void dijkstra(const Graph& graph, const string& source, const string& target) {
    map<string, int> distances;
    map<string, string> predecessors;
    priority_queue<pair<int, string>, vector<pair<int, string>>, CompareDist> pq;

    for (const auto& node : graph) {
        distances[node.first] = (node.first == source) ? 0 : numeric_limits<int>::max();
        pq.push({distances[node.first], node.first});
    }

    while (!pq.empty()) {
        string current_node = pq.top().second;
        pq.pop();

        if (current_node == target) break;

        for (const auto& neighbor : graph.at(current_node)) {
            string neighbor_node = neighbor.first;
            int edge_cost = neighbor.second;
            int new_distance = distances[current_node] + edge_cost;

            if (new_distance < distances[neighbor_node]) {
                distances[neighbor_node] = new_distance;
                predecessors[neighbor_node] = current_node;
                pq.push({new_distance, neighbor_node});
            }
        }
    }

    cout << "Shortest distance from " << source << " to " << target << " is " << distances[target] << endl;
    printPath(predecessors, target);
}

int main() {
    Graph graph = {
        {"S", {{"A", 7}, {"B", 2}, {"C", 3}}},
        {"A", {{"D", 4}, {"B", 3}, {"S", 7}}},
        {"B", {{"S", 2}, {"D", 4}, {"H", 1}, {"A", 3}}},
        {"D", {{"B", 4}, {"F", 5}, {"H", 4}}},
        {"H", {{"B", 1}, {"F", 3}, {"G", 2}}},
        {"F", {{"D", 5}, {"H", 3}}},
        {"G", {{"H", 2}, {"E", 2}}},
        {"C", {{"S", 3}, {"L", 2}}},
        {"L", {{"C", 2}, {"I", 4}, {"J", 4}}},
        {"I", {{"J", 6}, {"K", 4}, {"L", 4}}},
        {"J", {{"L", 4}, {"I", 6}, {"K", 4}}},
        {"K", {{"J", 4}, {"I", 4}, {"E", 5}}},
        {"E", {}}
    };

    // Specify source and target nodes
    string source = "S";
    string target = "E";

    dijkstra(graph, source, target);

    return 0;
}
