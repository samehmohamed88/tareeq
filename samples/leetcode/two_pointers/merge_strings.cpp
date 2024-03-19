#include <string>
#include <vector>
#include <iostream>

using namespace std;

class Solution {
public:
    string mergeAlternately(string word1, string word2) {
        size_t word1_Pointer = 0;
        size_t word2_Pointer = 0;

        std::vector<char> result;
        while (word1.size() > word1_Pointer && word2.size() > word2_Pointer) {
            result.push_back(word1[word1_Pointer]);
            result.push_back(word2[word2_Pointer]);
            ++word1_Pointer;
            ++word2_Pointer;
        }
        std::string word1Remainder = word1.substr(word1_Pointer, word1.size());
        std::string word2Remainder = word2.substr(word2_Pointer, word2.size());
        std::string str(result.begin(), result.end());
        return str + word1Remainder + word2Remainder;
    }
};

int main() {
    std::string word1 = "abc";
    std::string word2 = "def";

    std::cout << Solution().mergeAlternately(word1, word2) << std::endl;
}