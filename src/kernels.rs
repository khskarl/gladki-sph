use nalgebra::Vector2;

use std::f32::consts::PI;

type VectorN = Vector2<f32>;

pub fn spiky(r: VectorN, h: f32) -> VectorN {
  let coef = 45.0 / (PI * h.powi(6));
  let dist = r.magnitude();

  if h < dist {
    return VectorN::new(0.0, 0.0);
  }

  -coef * r.normalize() * (h - dist.powi(2))
}
