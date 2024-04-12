use std::collections::HashMap;
use std::error::Error;

struct Solution;

impl Solution {
    pub fn two_sums(nums: Vec<i32>, target: i32) -> Vec<i32> {
        let mut map : HashMap<i32, i32> = HashMap::new();
        let mut result : Vec<i32> = vec![];

        for i in 0..nums.len() {
            let compliment = target - nums[i];
            if let Some(&number) = map.get(&compliment) {
                result.push(number);
                result.push(i as i32);
            }
            map.insert(nums[i], i as i32);
        }
        result
    }
}

fn main() -> Result<(), Box<dyn Error>> {

    let nums : Vec<i32> = vec![2,7,11,15];
    println!("{:?}", Solution::two_sums(nums, 9));

    let nums : Vec<i32> = vec![3,2,4];
    println!("{:?}", Solution::two_sums(nums, 6));

    let nums : Vec<i32> = vec![3,3];
    println!("{:?}", Solution::two_sums(nums, 6));

    let nums : Vec<i32> = vec![3,2,3];
    println!("{:?}", Solution::two_sums(nums, 6));

    Ok(())
}