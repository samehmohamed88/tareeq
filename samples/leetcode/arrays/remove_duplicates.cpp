#include <vector>
#include <iostream>

using namespace std;

class Solution {
public:
    int removeDuplicates(vector<int>& nums) {
        int k = 1;
        int mem = nums[0];
        for (size_t i = 1 ; i < nums.size(); i++) {
            if (nums[i] > mem) {
                mem = nums[i];
                swap(nums[i], nums[k]);
                k++;
            }
        }
        return k;
    }
};

int main() {
    auto nums = vector<int>{0,0,1,1,1,2,2,3,3,4};
//    auto nums = vector<int>{1,1,2};
    auto solution = Solution();

    std::cout << solution.removeDuplicates(nums);
    return 0;
}