#include <string>
#include <stack>
#include <iostream>
#include <map>

using namespace std;

class Solution {
public:
    bool isValid(string s) {
        stack<char> stack_;
        unordered_map<char, char> map = {
            { ')' , '(', },
            { ']' , '[' },
            { '}' , '{' }
        };

        for (auto const& c : s) {
            if (map.contains(c)) {
                if (stack_.empty()) {return false;}
                if (stack_.top() != map[c]) {return false;}
                stack_.pop();
            } else {
                stack_.push(c);
            }
        }
        return stack_.empty();
    }
};

int main() {
    auto sol = Solution();
    std::string str("([)]");
    std::cout << std::boolalpha <<  "Returned " << sol.isValid(str) << std::endl;

    str = string("(]"); //
    std::cout << std::boolalpha <<  "Returned " << sol.isValid(str) << std::endl;

    str = string("(])");
    std::cout << std::boolalpha <<  "Returned " << sol.isValid(str) << std::endl;

    str = string("()[]{}");
    std::cout << std::boolalpha <<  "Returned " << sol.isValid(str) << std::endl;


    return 0;
}