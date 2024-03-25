#include <iostream>
#include <unordered_map>
#include <vector>
#include <deque>

class Solution {
public:
    bool hasPath(const std::unordered_map<char, std::vector<char>>& graph, char src, char dst) {
        std::deque<char> q{ src };
        while (!q.empty()) {
            auto current = q.front();
            q.pop_front();

            if (current == dst) { return true;}
            for (auto const c : graph.at(current)) {
                q.push_back(c);
            }
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

    std::cout << std::boolalpha << Solution().hasPath(graph, 'f', 'k') << std::endl;

    return 0;
}