mod waypoint;
mod routeplan;

pub fn add(left: usize, right: usize) -> usize {
    left + right
}

// Re-export routeplan module
pub use routeplan::RoutePlan;
// Re-export waypoint module
pub use waypoint::Waypoint;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
