use nalgebra::Vector3;

use crate::shape::Shape;

pub struct Object {
    pub shape: Shape,
    pub color: Vector3<f32>,
    pub specular: Option<f32>,
    pub reflection: f32,
    pub transparency: f32,
    pub refraction: f32,
}

impl Object {
    pub fn new(shape: Shape, color: Vector3<f32>, specular: Option<f32>, reflection: f32, transparency: f32, refraction: f32) -> Self {
        Self {
            shape,
            color,
            specular,
            reflection,
            transparency,
            refraction,
        }
    }
}

pub enum Light {
    Ambient(f32),
    Point(f32, Vector3<f32>),
    Directional(f32, Vector3<f32>),
}

