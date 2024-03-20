#include <vector>
#include <iostream>

using namespace std;

class Solution {
public:
    vector<int> sortArray(vector<int>& nums) {
        for (size_t i = 1; i < nums.size(); i++) {
            int j = i - 1;
            while (j >=0 && nums[j+1] < nums[j]) {
                swap(nums[j], nums[j+1]);
                j--;
            }
        }
        return nums;
    }
};


int main() {
    auto sol = Solution();

    auto nums = vector<int>{5,2,3,1}; // 5,1,1,2,0,0

    auto vec = sol.sortArray(nums);
    return 0;
}