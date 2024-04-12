#include <vector>
#include <iostream>

using namespace std;

class Solution {
public:
    int removeElement(vector<int>& nums, int val) {
        int n=nums.size();
        int count=0;
        for(int i=0;i<n;i++)
        {
            if(nums[i]!=val)
            {
                swap(nums[i],nums[count]);
                count++;
            }
        }

        return count;
    }
};

int main() {
    /**
    Input: nums = [0,1,2,2,3,0,4,2], val = 2
    Output: 5, nums = [0,1,4,0,3,_,_,_]
    Explanation: Your function should return k = 5, with the first five elements of nums containing 0, 0, 1, 3, and 4.
    Note that the five elements can be returned in any order.
    It does not matter what you leave beyond the returned k (hence they are underscores).
     */

    auto solution  = Solution();
    auto nums = std::vector<int>{0,1,2,2,3,0,4,2};
    auto val = 2;

    solution.removeElement(nums, val);


    return 0;
}
