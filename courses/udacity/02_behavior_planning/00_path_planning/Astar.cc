#include <iostream>
#include <map>
#include <queue>
#include <vector>
#include <limits>
#include <utility>
#include <unordered_map>
#include <cmath>
#include <algorithm>

using namespace std;

using Graph = map<string, vector<pair<string, int>>>;

// Function to represent the heuristic (estimated distance to the goal)
map<string, int> heuristic{
    {"S", 10}, {"A", 8},  {"B", 7},  {"C", 5},
    {"D", 6},  {"E", 0},  {"F", 4},  {"G", 2},
    {"H", 3},  {"I", 5},  {"J", 6},  {"K", 4},
    {"L", 7}
};

// Custom comparator for priority queue to sort based on the total cost (distance + heuristic)
class CompareDist {
public:
    bool operator()(pair<int, string> n1, pair<int, string> n2) {
        return n1.first > n2.first;
    }
};

void printPath(const map<string, string>& cameFrom, const string& start, const string& goal) {
    vector<string> path;
    string current = goal;
    while (current != start) {
        path.push_back(current);
        current = cameFrom.at(current);
    }
    path.push_back(start);

    reverse(path.begin(), path.end());

    cout << "Path: ";
    for (const auto& node : path) {
        cout << node << " ";
    }
    cout << endl;
}

void aStarSearch(const Graph& graph, const string& start, const string& goal) {
    priority_queue<pair<int, string>, vector<pair<int, string>>, CompareDist> openSet;
    map<string, int> gScore; // Cost from start to node
    map<string, int> fScore; // Total cost (gScore + heuristic)
    map<string, string> cameFrom;

    for (const auto& node : graph) {
        gScore[node.first] = numeric_limits<int>::max();
        fScore[node.first] = numeric_limits<int>::max();
    }

    gScore[start] = 0;
    fScore[start] = heuristic[start];
    openSet.push({fScore[start], start});

    while (!openSet.empty()) {
        string current = openSet.top().second;
        openSet.pop();

        if (current == goal) {
            printPath(cameFrom, start, goal);
            return;
        }

        for (const auto& neighbor : graph.at(current)) {
            string next = neighbor.first;
            int weight = neighbor.second;
            int tentative_gScore = gScore[current] + weight;

            if (tentative_gScore < gScore[next]) {
                cameFrom[next] = current;
                gScore[next] = tentative_gScore;
                fScore[next] = gScore[next] + heuristic[next];
                openSet.push({fScore[next], next});
            }
        }
    }

    cout << "Failed to find a path from " << start << " to " << goal << "." << endl;
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

    string start = "S";
    string goal = "E";

    aStarSearch(graph, start, goal);

    return 0;
}
