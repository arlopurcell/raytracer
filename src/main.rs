//! An e&xample of generating julia fractals.
extern crate image;
extern crate nalgebra;

use image::{RgbImage, Rgb};
use nalgebra::Vector3;

const VIEWPORT_WIDTH: f32 = 1.;
const VIEWPORT_HEIGHT: f32 = 1.;
const VIEWPORT_DISTANCE: f32 = 1.;

const CANVAS_WIDTH: i32 = 800;
const CANVAS_HEIGHT: i32 = 800;


fn main() {
    let mut image = RgbImage::new(CANVAS_WIDTH as u32, CANVAS_HEIGHT as u32);
    let camera = Vector3::new(0., 0., 0.);

    let mut scene = Scene::new(Vector3::new(1., 1., 1.));
    scene.spheres.push(Sphere::new(Vector3::new(0., -1., 3.), 1.0, Vector3::new(1., 0., 0.)));
    scene.spheres.push(Sphere::new(Vector3::new(2., 0., 4.), 1.0, Vector3::new(0., 0., 1.)));
    scene.spheres.push(Sphere::new(Vector3::new(-2., 0., 4.), 1.0, Vector3::new(0., 1., 0.)));
    scene.spheres.push(Sphere::new(Vector3::new(0., -5001., 0.), 5000., Vector3::new(1., 1., 0.)));

    scene.lights.push(Light::Ambient(0.2));
    scene.lights.push(Light::Point(0.6, Vector3::new(0., 0., 0.)));
    scene.lights.push(Light::Directional(0.2, Vector3::new(1., 4., 4.)));

    for x in -CANVAS_WIDTH/2..CANVAS_WIDTH/2 {
        for y in -CANVAS_HEIGHT/2..CANVAS_HEIGHT/2 {
            let direction = canvas_to_viewport(x, y);
            let color = scene.trace_ray(camera, direction, 1., f32::MAX);
            put_pixel(&mut image, x, y, color);
        }
    }

    image.save("render.png").unwrap();
}

fn put_pixel(image: &mut RgbImage, x: i32, y: i32, color: Vector3<f32>) {
    let x = x + CANVAS_WIDTH / 2;
    let y = -y + CANVAS_HEIGHT / 2;
    if x >= 0 && x < CANVAS_WIDTH && y >= 0 && y < CANVAS_HEIGHT {
        image.put_pixel(x as u32, y as u32, Rgb::from([
                                                      (color.x * 255.) as u8,
                                                      (color.y * 255.) as u8,
                                                      (color.z * 255.) as u8,
                                                      ]));
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
    lights: Vec<Light>,
    background: Vector3<f32>,
}

impl Scene {
    fn new(background: Vector3<f32>) -> Self {
        Self {
            spheres: Vec::new(),
            lights: Vec::new(),
            background,
        }
    }

    fn trace_ray(&self, camera: Vector3<f32>, direction: Vector3<f32>, t_min: f32, t_max: f32) -> Vector3<f32> {
        self.spheres.iter().filter_map(|sphere| {
            //let intersections = sphere.intersect_ray(&camera, &direction);
            let min_t = sphere.intersect_ray(&camera, &direction).filter(|t| *t > t_min && *t < t_max);
            min_t.map(|t| (sphere, t))
                /*
            let min_or_nan = intersections.into_iter().filter(|t| *t > t_min && *t < t_max)
                .fold(f32::NAN, f32::max);
            if min_or_nan.is_finite() {
                Some((sphere, min_or_nan))
            } else {
                None
            }
            */
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
        })
        .map(|(sphere, t)| {
            let p = camera + t * direction;
            let n = (p - sphere.center).normalize();
            sphere.color * self.compute_lighting(p, n)
        })
        .unwrap_or(self.background)
    }

    fn compute_lighting(&self, p: Vector3<f32>, n: Vector3<f32>) -> f32 {
        self.lights.iter().map(|light| match light {
            Light::Ambient(i) => *i,
            Light::Point(i, position) => {
                // looks like p is wrong somehow?
                let l = position - p;
                let n_dot_l = n.dot(&l).max(0.);
                i * n_dot_l / (n.magnitude() * l.magnitude())
            },
            Light::Directional(i, direction) => {
                let l = direction;
                let n_dot_l = n.dot(l).max(0.);
                i * n_dot_l / (n.magnitude() * l.magnitude())
            },
        }).sum()
    }
}

struct Sphere {
    center: Vector3<f32>,
    radius: f32,
    color: Vector3<f32>,
}

impl Sphere {
    fn new(center: Vector3<f32>, radius: f32, color: Vector3<f32>) -> Self {
        Self { center, radius, color }
    }

    fn intersect_ray(&self, camera: &Vector3<f32>, direction: &Vector3<f32>) -> Option<f32> {
        let co = camera - self.center;

        let a = direction.dot(direction);
        let b = 2. * co.dot(direction);
        let c = co.dot(&co) - self.radius * self.radius;
        solve_quadratic_min(a, b, c)
    }
}

fn solve_quadratic_min(a: f32, b: f32, c: f32) -> Option<f32> {
    let discriminant = b * b - 4. * a * c;
    let disc_sqrt = discriminant.sqrt();
    if !disc_sqrt.is_nan() {
        let root1 = (-b + disc_sqrt) / (2. * a);
        let root2 = (-b - disc_sqrt) / (2. * a);
        Some(root1.min(root2))
    } else {
        None
    }
}

enum Light {
    Ambient(f32),
    Point(f32, Vector3<f32>),
    Directional(f32, Vector3<f32>),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sphere_intersect() {
        let sphere = Sphere::new(Vector3::new(0., 0., 2.), 1., Vector3::new(0., 0., 0.));
        let camera = Vector3::new(0., 0., 0.);
        let direction = Vector3::new(0., 0., 1.);

        let t = sphere.intersect_ray(&camera, &direction);
        assert_eq!(Some(1.), t);

        let p = camera + direction;
        let n = (p - sphere.center).normalize();
        assert_eq!(Vector3::new(0., 0., -1.), n);

    }

    #[test]
    fn test_quad_solve() {
        assert_eq!(Some(-4.), solve_quadratic_min(1., 1., -12.));
    }
}
