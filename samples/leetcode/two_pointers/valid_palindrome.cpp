#include <string>
#include <cassert>

using namespace std;

class Solution {
public:
    bool isPalindrome(string s) {
        size_t j = s.size() -1;
        for (size_t i = 0; i < s.size()/2; i++) {
            if (s[i] != s[j]) {
                return false;
            }
            --j;
        }
        return true;
    }
};

int main() {
    auto solution = Solution();

    assert(solution.isPalindrome("amanaplanacanalpanama"));
    assert(!solution.isPalindrome("race a car"));

    return 0;
}