#[macro_use]
extern crate serde_derive;

use std::f32::consts::PI;
use std::ops::Rem;

use nalgebra::Vector2;
use wasm_bindgen::prelude::*;

mod spatial_hashmap;
use crate::spatial_hashmap::SpatialHashMap;

mod kernels;
mod wasm_utils;
use crate::wasm_utils::*;

#[derive(Serialize)]
struct SimulationData {
    pub positions: Vec<VectorN>,
}

#[derive(Serialize)]
struct DebugData {
    pub indices: Vec<usize>,
}

type VectorN = Vector2<f32>;

pub struct Distribution {
    pub theta_step: f32,
    pub radius_step: f32,
}

pub struct Particles {
    pub position: Vec<VectorN>,
    pub prev_position: Vec<VectorN>,
    pub velocity: Vec<VectorN>,
    pub acceleration: Vec<VectorN>,
    pub prev_acceleration: Vec<VectorN>,
    pub pressure: Vec<f32>,
    pub near_pressure: Vec<f32>,
    pub density: Vec<f32>,
}

impl Particles {
    pub fn new(count: usize, distribution: Distribution) -> Particles {
        let mut position = Vec::<VectorN>::new();
        let mut prev_position = Vec::<VectorN>::new();
        let mut velocity = Vec::<VectorN>::new();
        let mut acceleration = Vec::<VectorN>::new();
        let mut prev_acceleration = Vec::<VectorN>::new();
        let mut pressure = Vec::<f32>::new();
        let mut near_pressure = Vec::<f32>::new();
        let mut density = Vec::<f32>::new();

        for i in 0..count {
            let theta = i as f32 * distribution.theta_step;
            let magical_theta = (theta / (PI * 2.0)).floor();
            let radius = distribution.radius_step * magical_theta + 0.2;

            let x = radius * theta.cos();
            let y = radius * theta.sin();

            position.push(VectorN::new(x, y));
            prev_position.push(VectorN::new(0.0, 0.0));
            velocity.push(VectorN::new(0.0, 0.0));
            acceleration.push(VectorN::new(0.0, 0.0));
            prev_acceleration.push(VectorN::new(0.0, 0.0));
            pressure.push(0.0);
            near_pressure.push(0.0);
            density.push(0.0);
        }

        Particles {
            position,
            prev_position,
            velocity,
            acceleration,
            prev_acceleration,
            pressure, //TODO rename para density
            near_pressure,
            density,
        }
    }

    pub fn count(&self) -> usize {
        self.position.len()
    }
}

///
///
///
///
pub struct SimulationParameters {
    pub smoothing_radius: f32,
    pub rest_density: f32,
    pub stiffness: f32,
    pub stiffness_near: f32,
    pub gravity: VectorN,
}

#[wasm_bindgen]
pub struct Simulation {
    particles: Particles,
    radius: f32,
    hashmap: SpatialHashMap,
    params: SimulationParameters,
}

#[wasm_bindgen]
impl Simulation {
    #[wasm_bindgen(constructor)]
    pub fn new(
        particles_count: usize,
        radius: f32,
        params: SimulationParameters,
        distribution: Distribution,
    ) -> Simulation {
        Simulation {
            particles: Particles::new(particles_count, distribution),
            hashmap: SpatialHashMap::new(
                radius * 2.0,
                radius * 2.0,
                params.smoothing_radius,
                params.smoothing_radius,
            ),
            radius,
            params,
        }
    }

    pub fn particle_count(&self) -> usize {
        self.particles.count()
    }

    pub fn get_particles_positions(&self) -> Vec<VectorN> {
        self.particles.position.clone()
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
        // Prepare particles by inserting their index into the hashmap
        // and updating their position from the previous frame
        self.hashmap.clear();
        for i in 0..self.particles.count() {
            // Update spatial hashmap
            self.hashmap.insert(
                self.particles.position[i].x,
                self.particles.position[i].y,
                i,
            );
        }

        //
        for i in 0..self.particles.count() {
            let mut density = 0.0;
            let mut near_density = 0.0;

            for j in self.find_neighbors(i) {
                if i == j {
                    continue;
                }

                let distance_squared =
                    (self.particles.position[i] - self.particles.position[j]).magnitude_squared();

                if distance_squared < self.params.smoothing_radius.powi(2) {
                    let coefficient = 1.0 - distance_squared.sqrt() / self.params.smoothing_radius;
                    density += coefficient.powi(2);
                    near_density += coefficient.powi(3);
                }
            }

            let params = &self.params;
            self.particles.pressure[i] = params.stiffness * (density - params.rest_density);
            self.particles.near_pressure[i] = params.stiffness_near * near_density;
        }

        for i in 0..self.particles.count() {
            let mut pressure = VectorN::new(0.0, 0.0);
            // let mut viscosity = VectorN::new(0.0, 0.0);

            for j in self.find_neighbors(i) {
                if i == j {
                    continue;
                }
                let r = self.particles.position[j] - self.particles.position[i];

                let q = r.magnitude() / self.params.smoothing_radius;
                if q > 1.0 {
                    continue;
                }
                let g = 1.0 - q;

                let a = self.particles.pressure[i] * g;
                let b = self.particles.near_pressure[i] * g.powi(2);
                let d = r.normalize() * (a + b);
                pressure += d / 2.0;
                self.particles.acceleration[j] += d / 2.0;
            }
            self.particles.acceleration[i] = -pressure + self.params.gravity;
        }

        for i in 0..self.particles.count() {
            self.particles.velocity[i] += self.particles.acceleration[i] * dt;

            self.particles.position[i] += self.particles.velocity[i] * dt;

            if self.particles.position[i].magnitude_squared() > self.radius.powi(2) {
                self.particles.position[i] = self.particles.position[i].normalize() * self.radius;
                self.particles.velocity[i] *= -0.2;
            }

            self.particles.prev_acceleration[i] = self.particles.acceleration[i];
            self.particles.prev_position[i] = self.particles.position[i];

            self.particles.acceleration[i] = VectorN::new(0.0, 0.0);
        }
    }

    pub fn find_neighbors(&self, i: usize) -> Vec<usize> {
        self.hashmap
            .query(self.particles.position[i].x, self.particles.position[i].y)
    }
}
