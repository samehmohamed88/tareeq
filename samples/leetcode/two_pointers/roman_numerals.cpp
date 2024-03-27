#include <unordered_map>
#include <iostream>
#include <string>

class Solution {
public:
    int romanToInt(std::string s) {
        auto const map = std::unordered_map<std::string, int> {
            { "I" , 1},
            { "V" , 5},
            { "X" , 10},
            { "L" , 50},
            { "C" , 100},
            { "D" , 500},
            { "M" , 1000},
            { "IV" , 4},
            { "IX" , 9},
            { "XL" , 40},
            { "XC" , 90},
            { "CD" , 400},
            { "CM" , 900}
        };

        int i = 0;
        int sum = 0;
        while (i < s.size()) {
            if ( (i < s.size() -1) && map.contains(s.substr(i,2))) {
                sum += map.at(s.substr(i,2));
                i += 2;
            } else {
                sum += map.at(s.substr(i,1));
                ++i;
            }
        }
        return sum;
    }
};

int main() {
    auto sol = Solution();
    std::cout << sol.romanToInt("III");
    return 0;
}