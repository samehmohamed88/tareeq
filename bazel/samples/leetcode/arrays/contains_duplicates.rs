use std::collections::HashMap;
use std::error::Error;

struct Solution;

impl Solution {

    fn using_hash_map(nums: Vec<i32>) -> bool {
        let mut map = HashMap::new();
        for &num in nums.iter() {
            if map.contains_key(&num) {
                return true;
            }
            map.insert(num, 1);
        }
        return false;
    }
    pub fn contains_duplicate(nums: Vec<i32>) -> bool {
        return Self::using_hash_map(nums);
    }
}
fn main() -> Result<(), Box<dyn Error>> {
    let nums = vec![1,2,3,1];
    println!("{}", Solution::contains_duplicate(nums));
    let nums = vec![1,1,1,3,3,4,3,2,4,2];
    println!("{}", Solution::contains_duplicate(nums));
    let nums = vec![1,2,3,4];
    println!("{}", Solution::contains_duplicate(nums));

    Ok(())
}
