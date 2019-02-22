#[macro_use]
extern crate serde_derive;

use std::f32::consts::PI;
use std::ops::Rem;

use nalgebra::Vector2;
use wasm_bindgen::prelude::*;

mod spatial_hashmap;
use crate::spatial_hashmap::SpatialHashMap;

#[derive(Serialize)]
struct SimulationData {
    pub positions: Vec<VectorN>,
}

#[derive(Serialize)]
struct DebugData {
    pub indices: Vec<usize>,
}

type VectorN = Vector2<f32>;

pub struct Particles {
    pub position: Vec<VectorN>,
    pub prev_position: Vec<VectorN>,
    pub velocity: Vec<VectorN>,
    pub pressure: Vec<f32>,
    pub near_pressure: Vec<f32>,
    pub gradient: Vec<f32>,
}

impl Particles {
    pub fn new(count: usize, theta_step: f32, radius_step: f32) -> Particles {
        let mut position = Vec::<VectorN>::new();
        let mut prev_position = Vec::<VectorN>::new();
        let mut velocity = Vec::<VectorN>::new();
        let mut pressure = Vec::<f32>::new();
        let mut near_pressure = Vec::<f32>::new();
        let mut gradient = Vec::<f32>::new();

        for i in 0..count {
            let theta = i as f32 * theta_step;
            let magical_theta = (theta / (PI * 2.0)).floor();
            let radius = radius_step * magical_theta + 0.2;

            let x = radius * theta.cos();
            let y = radius * theta.sin();

            position.push(VectorN::new(x, y));
            prev_position.push(VectorN::new(0.0, 0.0));
            velocity.push(VectorN::new(0.0, 0.0));
            pressure.push(0.0);
            near_pressure.push(0.0);
            gradient.push(0.0);
        }

        Particles {
            position,
            prev_position,
            velocity,
            pressure,
            near_pressure,
            gradient,
        }
    }

    pub fn count(&self) -> usize {
        self.position.len()
    }
}

struct SimulationParameters {}

#[wasm_bindgen]
pub struct Simulation {
    particles: Particles,
    gravity: VectorN,
    hashmap: SpatialHashMap,
    width: f32,
    height: f32,
    params: SimulationParameters,
}

#[wasm_bindgen]
impl Simulation {
    #[wasm_bindgen(constructor)]
    pub fn new(
        particles_count: usize,
        width: f32,
        height: f32,
        neighbor_radius: f32,
        theta_step: f32,
        radius_step: f32,
    ) -> Simulation {
        let params = SimulationParameters {};

        Simulation {
            particles: Particles::new(particles_count, theta_step, radius_step),
            gravity: VectorN::new(0.0, -9.8),
            hashmap: SpatialHashMap::new(width, height, neighbor_radius, 10.0),
            width,
            height,
            params,
        }
    }

    pub fn particle_count(&self) -> usize {
        self.particles.count()
    }

    /// Serialize and send the simulation state to JavaScript
    pub fn send_simulation_to_js(&self) -> JsValue {
        let simulation_data = SimulationData {
            positions: self.particles.position.clone(),
        };

        JsValue::from_serde(&simulation_data).unwrap()
    }

    /// Serialize and send debug_data to JavaScript
    pub fn send_debug_to_js(&self, x: f32, y: f32) -> JsValue {
        let debug_data = DebugData {
            indices: self.hashmap.query(x, y),
        };

        JsValue::from_serde(&debug_data).unwrap()
    }

    pub fn step(&mut self, dt: f32) {
        self.hashmap.clear();

        let particles = &mut self.particles;
        for i in 0..particles.count() {
            particles.prev_position[i] = particles.position[i];

            // Apply forces
            // particles.velocity[i] += self.gravity * dt;

            // Apply velocity
            particles.position[i] += particles.velocity[i] * dt;

            // Update spatial hashmap
            {
                // let i
                self.hashmap
                    .insert(particles.position[i].x, particles.position[i].y, i);
            }
        }
    }

    fn apply_pressure(&mut self, p0: usize, p1: usize) {
        // let dividend = self.particles.pressure[p0] + self.particles.pressure[p1];
        // let divisor = 2 * self.particles.density[p0] * self.particles.density[p1];

        // let r = self.particles.position[p0] - self.particles.position[p1];
        // return -1.0 * (dividend / divisor) * Kernels.GradientSpiky(r, smoothingRadius);
    }
}
