#include <vector>
#include <iostream>

using namespace std;

class Solution {
public:
    int search(vector<int>& nums, int target) {
        int low = 0;
        int high = nums.size() - 1;
        while (low <= high) {
            int mid = low + (high - low) / 2;
            if (target == nums[mid]) {
                return mid;
            }
            if (target > nums[mid]) {
                low = mid + 1;
            } else {
                high = mid - 1;
            }
        }
        return -1;
    }
};

int main() {
    /**
    Input: nums = [-1,0,3,5,9,12], target = 9
    Output: 4
     */

    auto sol = Solution();

    int target = 2;
    auto nums = vector<int>{-1,0,3,5,9,12};
    std::cout << sol.search(nums, target) << std::endl;

    return 0;
}