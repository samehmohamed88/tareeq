#include <string>
#include <iostream>
#include <unordered_map>
#include <cassert>

using namespace std;

class Solution {
public:
    bool isAnagram(string s, string t) {
        auto s_map = unordered_map<char, int>{};
        auto t_map = unordered_map<char, int>{};
        for (const auto &c : s) {
            if (s_map.contains(c)) {
                ++s_map[c];
            } else {
                s_map[c] = 0;
            }
        }
        for (const auto &c : t) {
            if (t_map.contains(c)) {
                ++t_map[c];
            } else {
                t_map[c] = 0;
            }
        }
        for (auto [key,value] : s_map) {
            if (!t_map.contains(key)) return false;
            if (s_map.at(key) != t_map.at(key)) return false;
        }
        return true;
    }
};

int main() {
    std::string s("anagram");
    std::string t("nagaram");

    auto solution = Solution();

    assert(solution.isAnagram(s, t));

    s = "rat";
    t = "car";

    assert(!solution.isAnagram(s, t));
    
    return 0;
}