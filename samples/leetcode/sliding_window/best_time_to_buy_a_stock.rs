use std::error::Error;
use std::cmp::max;

struct Solution;

impl Solution {
    pub fn max_profit(prices: Vec<i32>) -> i32 {
        let mut buy_index: usize = 0;
        let mut max_profit = 0;

        for sell_index in 1..prices.len() {
            if prices[sell_index] < prices[buy_index] {
                buy_index = sell_index;
            }
            max_profit = max(max_profit, prices[sell_index] - prices[buy_index]);
        }
        max_profit
    }
}

fn main() -> Result<(), Box<dyn Error>> {

    let vec = vec![7,1,5,3,6,4];
    println!("{}", Solution::max_profit(vec));

    Ok(())
}