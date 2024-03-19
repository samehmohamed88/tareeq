use std::error::Error;

struct Solution;

impl Solution {
    pub fn is_palindrome(s: String) -> bool {
        let chars: Vec<char> = s.chars().collect();
        let mut j = chars.len().saturating_sub(1); // Prevent underflow
        for i in 0..chars.len() / 2 { // Only need to iterate through half
            let a = chars[i];
            let b = chars[j];

            if a != b {
                return false;
            }

            if j == 0 {
                break; // Prevent underflow
            }
            j -= 1;
        }
        true
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    assert!(Solution::is_palindrome("amanaplanacanalpanama".to_string()));
    assert!(!Solution::is_palindrome("race a car".to_string()));
    Ok(())
}