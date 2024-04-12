#include <vector>
#include <iostream>
#include <unordered_map>

using namespace std;

class Solution {
public:

    bool usingHashMap(vector<int>& nums) {
        std::unordered_map<int, int> unorderedMap;
        for (auto &x : nums) {
            if (unorderedMap.contains(x)) {
                return true;
            }
            unorderedMap[x] = 1;
        }
        return false;
    }

    bool containsDuplicate(vector<int>& nums) {
        return usingHashMap(nums);
    }
};

int main() {
    //vector<int> nums = {1,2,3,1};
    //vector<int> nums = {1,1,1,3,3,4,3,2,4,2};
    vector<int> nums = {1,2,3,4};
    auto s = Solution();
    auto ret = s.containsDuplicate(nums);
    std::cout << std::boolalpha <<  "Returned " << ret << std::endl;
    return 0;
}