#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
    vector<int> twoSum(const vector<int>& nums, int target) {
        std::vector<int> result;
        std::unordered_map<int, int> numMap;  // Map to store number and its index

        for (size_t i = 0; i < nums.size(); ++i) {
            int complement = target - nums[i];

            if (numMap.find(complement) != numMap.end()) {
                result.push_back(numMap[complement]);
                result.push_back(i);
                return result;  // Return immediately once the pair is found
            }

            numMap[nums[i]] = i;  // Add the current number to the map
        }

        return result;  // Return an empty vector if no pair is found
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

    nums =  {3,2,3};
    target = 6;

    vec = solution.twoSum(nums, target);
    std::ranges::for_each(vec, [](const int& element) {
        std::cout << element << " ";
    });
    std::cout << std::endl;


    return 0;
}