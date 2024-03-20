#include <vector>
#include <span>
#include <iostream>

using namespace std;

#include <vector>
#include <iterator>

class Solution {
public:
    std::vector<int> merge(const std::vector<int>& leftArray, const std::vector<int>& rightArray) {
        std::vector<int> mergedArray;
        mergedArray.reserve(leftArray.size() + rightArray.size()); // Optimize memory allocation

        auto leftIter = leftArray.begin();
        auto rightIter = rightArray.begin();

        while (leftIter != leftArray.end() && rightIter != rightArray.end()) {
            if (*leftIter < *rightIter) {
                mergedArray.push_back(*leftIter++);
            } else {
                mergedArray.push_back(*rightIter++);
            }
        }

        // Append remaining elements (if any)
        std::copy(leftIter, leftArray.end(), std::back_inserter(mergedArray));
        std::copy(rightIter, rightArray.end(), std::back_inserter(mergedArray));

        return mergedArray;
    }

    std::vector<int> sortArray(std::vector<int>& nums) {
        if (nums.size() <= 1) {
            return nums;
        }

        auto mid = nums.begin() + nums.size() / 2;
        std::vector<int> leftArray(nums.begin(), mid);
        std::vector<int> rightArray(mid, nums.end());

        leftArray = sortArray(leftArray);
        rightArray = sortArray(rightArray);

        return merge(leftArray, rightArray);
    }
};

int main() {
    auto sol = Solution();

    auto nums = vector<int>{5,2,3,1}; // 5,1,1,2,0,0

    auto vec = sol.sortArray(nums);
    return 0;
}