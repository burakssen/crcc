pub struct Circle {
    pub(super) center: (f64, f64),
    pub(super) radius: f64,
}

impl Circle {
    pub fn new(center: (f64, f64), radius: f64) -> Circle {
        if radius <= 0.0 {
            panic!("Circle radius must be positive.");
        }
        Circle { center, radius }
    }

    pub fn center(&self) -> (f64, f64) {
        self.center
    }

    pub fn radius(&self) -> f64 {
        self.radius
    }
}
