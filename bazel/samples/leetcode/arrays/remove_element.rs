use std::error::Error;


impl Solution {
    pub fn remove_element(nums: &mut Vec<i32>, val: i32) -> i32 {
        let mut count : usize = 0;
        for i in 0..nums.len() {
            if (nums[i] != val) {
                nums.swap(i, count);
                count += 1;
            }
        }
        count as i32
    }
}

fn main() -> Result<(), Box<dyn Error>> {

}