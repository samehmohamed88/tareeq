use crate::waypoint::Waypoint;

pub struct RoutePlan {
    waypoints: Vec<Waypoint>,
}

impl RoutePlan {
    pub fn new() -> Self {
        RoutePlan {
            waypoints: Vec::new(),
        }
    }

    pub fn add_waypoint(&mut self, waypoint: Waypoint) {
        self.waypoints.push(waypoint);
    }

    // Add other methods to manipulate the route plan as needed
}
