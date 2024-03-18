use std::collections::HashMap;
use std::error::Error;

struct Solution;

impl Solution {
    pub fn is_anagram(s: String, t: String) -> bool {
        let mut s_map: HashMap<char, i32> = HashMap::new();
        let mut t_map: HashMap<char, i32> = HashMap::new();

        for c in s.chars() {
            let count = s_map.entry(c).or_insert(1);
            *count += 1;
        }

        for c in t.chars() {
            let count = t_map.entry(c).or_insert(1);
            *count += 1;
        }

        // Check if s_map and t_map have the same keys with the same values
        if s_map.len() != t_map.len() {
            return false;
        }

        for (key, value) in &s_map {
            if t_map.get(key) != Some(value) {
                return false;
            }
        }

        true
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let s : String = String::from("anagram");
    let t : String = String::from("nagaram");

    assert!(Solution::is_anagram(s, t), "solution is incorrect");

    Ok(())
}