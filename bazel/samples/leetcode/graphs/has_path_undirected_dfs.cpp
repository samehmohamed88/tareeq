#include <iostream>
#include <unordered_map>
#include <array>
#include <vector>
#include <unordered_set>

auto PrintGraph(const std::unordered_map<char, std::vector<char>>& graph) {
    std::cout << "{" << std::endl;
    for (const auto& [key, values] : graph) {
        std::cout << key << " : [";
        for (const auto c : values) {
            std::cout << c << " ,";
        }
        std::cout << "]" << std::endl;
    }
    std::cout << "}" << std::endl;
}

class Solution {
public:
    bool hasPath(const std::unordered_map<char, std::vector<char>>& graph, char src, char dst) {
        if (src == dst) { return true;}
        if (visited_.contains(src)) {return false;}
        visited_.insert(src);

        for (auto const c : graph.at(src)) {
            if (hasPath(graph, c, dst)) { return true;}
        }
        return false;
    }
private:
    std::unordered_set<char> visited_;
};

auto buildGraph(const std::array<std::array<char, 2>, 6>& edges) -> std::unordered_map<char, std::vector<char>>  {
    std::unordered_map<char, std::vector<char>> graph;
    for (auto const edge : edges) {
        auto const a = edge[0];
        auto const b = edge[1];
        graph[a].push_back(b);
        graph[b].push_back(a);
    }
    return graph;
};

int main() {
    std::array<std::array<char, 2>, 6> edges{{
        {'i','j'},
        {'k', 'j'},
        {'k', 'i'},
        {'m', 'k'},
        {'k', 'l'},
        {'o', 'n'}
    }};

    auto graph = buildGraph(edges);
    PrintGraph(graph);

    std::cout << std::boolalpha << Solution().hasPath(graph, 'i', 'n') << std::endl;

    return 0;
}