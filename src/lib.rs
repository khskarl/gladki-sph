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
    pub fn new(count: usize, theta_step: f32, radius_step: f32) -> Particles {
        let mut position = Vec::<VectorN>::new();
        let mut prev_position = Vec::<VectorN>::new();
        let mut velocity = Vec::<VectorN>::new();
        let mut acceleration = Vec::<VectorN>::new();
        let mut prev_acceleration = Vec::<VectorN>::new();
        let mut pressure = Vec::<f32>::new();
        let mut near_pressure = Vec::<f32>::new();
        let mut density = Vec::<f32>::new();

        for i in 0..count {
            let theta = i as f32 * theta_step;
            let magical_theta = (theta / (PI * 2.0)).floor();
            let radius = radius_step * magical_theta + 0.2;

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
            pressure,
            near_pressure,
            density,
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
            hashmap: SpatialHashMap::new(width, height, neighbor_radius, 5.0),
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

            for j in self.find_neighbors(i) {
                if i == j {
                    continue;
                }

                let distance_squared =
                    (self.particles.position[i] - self.particles.position[j]).magnitude_squared();

                let smoothing_radius = 10.0;
                if distance_squared < smoothing_radius * smoothing_radius {
                    density += (1.0 - distance_squared.sqrt() / smoothing_radius).powi(2);
                }
            }

            let rest_density = 82.0;
            let stiffness = 5000.0;
            // console_log!("Desnity: {}", density);

            self.particles.density[i] = density.max(rest_density);
            self.particles.pressure[i] = stiffness * (density - rest_density);
        }

        for i in 0..self.particles.count() {
            let mut pressure = VectorN::new(0.0, 0.0);
            let mut viscosity = VectorN::new(0.0, 0.0);

            // console_log!("self.find_neighbors(i): {:?}", self.find_neighbors(i));

            for j in self.find_neighbors(i) {
                if i == j {
                    continue;
                }

                // Pressure gradient
                pressure += {
                    let dividend = self.particles.pressure[i] + self.particles.pressure[j];
                    let divisor = 2.0 * self.particles.density[i] + self.particles.density[j];
                    // console_log!("Dividend: {}, Divisor: {}", dividend, divisor);

                    let r = self.particles.position[j] - self.particles.position[i];
                    // console_log!("r: {}", r.magnitude());
                    let smoothing_radius = 10.0;

                    1.0 * (dividend / divisor) * kernels::spiky(r, smoothing_radius)
                }
            }
            // console_log!("Pressure: {}", pressure);
            self.particles.acceleration[i] = pressure + viscosity;
        }

        for i in 0..self.particles.count() {
            self.particles.velocity[i] +=
                0.5 * (self.particles.prev_acceleration[i] + self.particles.acceleration[i]) * dt;

            self.particles.position[i] +=
                self.particles.velocity[i] * dt + 0.5 * self.particles.acceleration[i] * dt * dt;

            if self.particles.position[i].magnitude_squared() > self.width.powi(2) {
                self.particles.position[i] = self.particles.position[i].normalize() * self.width;
            }

            self.particles.prev_acceleration[i] = self.particles.acceleration[i];
            self.particles.prev_position[i] = self.particles.position[i];
        }
    }

    fn find_neighbors(&self, i: usize) -> Vec<usize> {
        self.hashmap
            .query(self.particles.position[i].x, self.particles.position[i].y)
    }
}
