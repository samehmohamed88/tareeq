#include <vector>
#include <iostream>

using namespace std;

class Solution {
public:
    int maxProfit(vector<int>& prices) {
        size_t buyIndex = 0;
        int maxProfit = 0;
        for (size_t sellIndex = 1; sellIndex < prices.size(); sellIndex++) {
            if (prices[sellIndex] < prices[buyIndex]) {
                buyIndex = sellIndex;
                continue;
            }
            maxProfit = std::max(maxProfit, prices[sellIndex] - prices[buyIndex]);
        }
        return maxProfit;
    }
};

int main() {
    /**
    Input: prices = [7,1,5,3,6,4]
    Output: 5
    Explanation: Buy on day 2 (price = 1) and sell on day 5 (price = 6), profit = 6-1 = 5.
    Note that buying on day 2 and selling on day 1 is not allowed because you must buy before you sell.
     */

    auto solution = Solution();
    auto prices = vector<int>{7,1,5,3,6,4};
    std::cout << solution.maxProfit(prices) << std::endl;
    return 0;
}
