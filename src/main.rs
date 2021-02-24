mod object;
mod shape;
mod scene;

use image::{Rgb, RgbImage};
use nalgebra::{geometry::Rotation3, Vector3, Point3};
use rayon::prelude::*;

use crate::scene::Scene;

const VIEWPORT_WIDTH: f32 = 2.;
const VIEWPORT_HEIGHT: f32 = 1.;
const VIEWPORT_DISTANCE: f32 = 1.;

const CANVAS_WIDTH: i32 = 1800;
const CANVAS_HEIGHT: i32 = 800;

fn main() {
    let camera = Point3::new(0., 0., -1.);
    let camera_rotation = Rotation3::face_towards(
        &Vector3::new(0., 0., 1.), // direction
        &Vector3::new(0., 1., 0.),   // up
    );

    let scene = Scene::sample_scene_1();

    let mut pixels = Vec::new();
    for x in -CANVAS_WIDTH / 2..CANVAS_WIDTH / 2 {
        for y in -CANVAS_HEIGHT / 2..CANVAS_HEIGHT / 2 {
            pixels.push((x, y));
        }
    }

    let pixels: Vec<_> = pixels
        .into_par_iter()
        .map(|(x, y)| {
            let direction = camera_rotation * canvas_to_viewport(x, y);
            let color = scene.trace_ray(&camera, &direction, 1., f32::MAX, 1.0, 3);
            (x, y, color)
        })
        .collect();

    let mut image = RgbImage::new(CANVAS_WIDTH as u32, CANVAS_HEIGHT as u32);
    for (x, y, color) in pixels {
        put_pixel(&mut image, x, y, color);
    }
    image.save("render.png").unwrap();
}

fn put_pixel(image: &mut RgbImage, x: i32, y: i32, color: Vector3<f32>) {
    let x = x + CANVAS_WIDTH / 2;
    let y = -y + CANVAS_HEIGHT / 2;
    if x >= 0 && x < CANVAS_WIDTH && y >= 0 && y < CANVAS_HEIGHT {
        image.put_pixel(
            x as u32,
            y as u32,
            Rgb::from([
                (color.x.min(1.).max(0.) * 255.) as u8,
                (color.y.min(1.).max(0.) * 255.) as u8,
                (color.z.min(1.).max(0.) * 255.) as u8,
            ]),
        );
    }
}

fn canvas_to_viewport(x: i32, y: i32) -> Vector3<f32> {
    Vector3::new(
        x as f32 * VIEWPORT_WIDTH / CANVAS_WIDTH as f32,
        y as f32 * VIEWPORT_HEIGHT / CANVAS_HEIGHT as f32,
        VIEWPORT_DISTANCE,
    )
}
