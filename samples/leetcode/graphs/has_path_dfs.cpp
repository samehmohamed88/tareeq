#include <iostream>
#include <unordered_map>
#include <vector>

class Solution {
public:
    bool hasPath(const std::unordered_map<char, std::vector<char>>& graph, char src, char dst) {
        if (src == dst) { return true;}
        for (auto const c : graph.at(src)) {
            if (hasPath(graph, c, dst)) { return true;}
        }
        return false;
    }
};

int main() {
    std::unordered_map<char, std::vector<char>> graph = {
        {'f' , {'g', 'i'} },
        {'g' , {'h'}},
        {'i' , {'g', 'k'}},
        {'j' , {'i'}},
        {'k' , {}},
        {'h' , {}}
    };

    std::cout << std::boolalpha << Solution().hasPath(graph, 'f', 'j') << std::endl;

    return 0;
}