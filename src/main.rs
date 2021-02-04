//! An e&xample of generating julia fractals.
extern crate image;
extern crate nalgebra;

use image::{RgbImage, Rgb};
use nalgebra::{Vector2, Vector3};

const VIEWPORT_WIDTH: f32 = 1.;
const VIEWPORT_HEIGHT: f32 = 1.;
const VIEWPORT_DISTANCE: f32 = 1.;

const CANVAS_WIDTH: i32 = 800;
const CANVAS_HEIGHT: i32 = 800;


fn main() {
    let mut image = RgbImage::new(CANVAS_WIDTH as u32, CANVAS_HEIGHT as u32);
    let camera = Vector3::new(0., 0., 0.);

    let mut scene = Scene::default();
    scene.spheres.push(Sphere::new(Vector3::new(0., -1., 3.), 1.0, Rgb::from([255, 0, 0])));
    scene.spheres.push(Sphere::new(Vector3::new(2., 0., 4.), 1.0, Rgb::from([0, 0, 255])));
    scene.spheres.push(Sphere::new(Vector3::new(-2., 0., 4.), 1.0, Rgb::from([0, 255, 0])));

    for x in -CANVAS_WIDTH/2..CANVAS_WIDTH/2 {
        for y in -CANVAS_HEIGHT/2..CANVAS_HEIGHT/2 {
            let direction = canvas_to_viewport(x, y);
            let color = scene.trace_ray(camera, direction, 1., f32::MAX);
            put_pixel(&mut image, x, y, color);
        }
    }

    image.save("render.png").unwrap();
}

fn put_pixel(image: &mut RgbImage, x: i32, y: i32, color: Rgb<u8>) {
    let x = x + CANVAS_WIDTH / 2;
    let y = -y + CANVAS_HEIGHT / 2;
    if x >= 0 && x < CANVAS_WIDTH && y >= 0 && y < CANVAS_HEIGHT {
        image.put_pixel(x as u32, y as u32, color);
    }
}

fn canvas_to_viewport(x: i32, y: i32) -> Vector3<f32> {
    Vector3::new(
        x as f32 * VIEWPORT_WIDTH / CANVAS_WIDTH as f32,
        y as f32 * VIEWPORT_HEIGHT / CANVAS_HEIGHT as f32,
        VIEWPORT_DISTANCE,
    )
}

#[derive(Default)]
struct Scene {
    spheres: Vec<Sphere>,
}

impl Scene {
    fn trace_ray(&self, camera: Vector3<f32>, direction: Vector3<f32>, t_min: f32, t_max: f32) -> Rgb<u8> {
        self.spheres.iter().filter_map(|sphere| {
            let intersections = sphere.intersect_ray(&camera, &direction);
            let min_or_nan = intersections.into_iter().filter(|t| *t > t_min && *t < t_max)
                .fold(f32::NAN, f32::min);
            if min_or_nan.is_finite() {
                Some((sphere, min_or_nan))
            } else {
                None
            }
        }).fold(None, |acc, (sphere, t)| {
            if let Some((_min_sphere, min_t)) = acc {
                if t < min_t {
                    Some((sphere, t))
                } else {
                    acc
                }
            } else {
                Some((sphere, t))
            }
        }).map(|(sphere, _t)| sphere.color).unwrap_or(Rgb::from([0, 0, 0]))
    }
}

struct Sphere {
    center: Vector3<f32>,
    radius: f32,
    color: Rgb<u8>,
}

impl Sphere {
    fn new(center: Vector3<f32>, radius: f32, color: Rgb<u8>) -> Self {
        Self { center, radius, color }
    }

    fn intersect_ray(&self, camera: &Vector3<f32>, direction: &Vector3<f32>) -> Vec<f32> {
        let co = camera - self.center;

        let a = direction.dot(direction);
        let b = 2. * co.dot(direction);
        let c = co.dot(&co) - self.radius * self.radius;

        let discriminant = b * b - 4. * a * c;
        let mut roots = Vec::new();
        if discriminant == 0. {
            roots.push(-b / (2. * a));
        } else if discriminant > 0. {
            let disc_sqrt = discriminant.sqrt();
            roots.push(-b + disc_sqrt / (2. * a));
            roots.push(-b - disc_sqrt / (2. * a));
        }
        roots
    }
}

