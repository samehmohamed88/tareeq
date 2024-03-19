#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
    vector<int> twoSum(vector<int>& nums, int target) {
        std::vector<int> result;
        size_t indexOne = 0;
        size_t indexTwo = 1;
        while (indexTwo < nums.size()) {
            int numToAdd = target - nums[indexOne];
            if (nums[indexTwo] == numToAdd) {
                result.push_back(indexOne);
                result.push_back(indexTwo);
                break;
            }
            ++indexOne;
            ++indexTwo;
        }
        return result;
    }
};

int main() {
    /**
     *  Input: nums = [2,7,11,15], target = 9
        Output: [0,1]
        Explanation: Because nums[0] + nums[1] == 9, we return [0, 1].
     */

    auto solution = Solution();

    std::vector<int> nums =  {2,7,11,15};
    int target = 9;

    auto vec = solution.twoSum(nums, target);
    std::ranges::for_each(vec, [](const int& element) {
        std::cout << element << " ";
    });
    std::cout << std::endl;

    nums =  {3,2,4};
    target = 6;

    vec = solution.twoSum(nums, target);
    std::ranges::for_each(vec, [](const int& element) {
        std::cout << element << " ";
    });
    std::cout << std::endl;

    nums =  {3,3};
    target = 6;

    vec = solution.twoSum(nums, target);
    std::ranges::for_each(vec, [](const int& element) {
        std::cout << element << " ";
    });
    std::cout << std::endl;


    return 0;
}