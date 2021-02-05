use image::{Rgb, RgbImage};
use nalgebra::{geometry::Rotation3, Vector3};
use rayon::prelude::*;

const VIEWPORT_WIDTH: f32 = 2.;
const VIEWPORT_HEIGHT: f32 = 1.;
const VIEWPORT_DISTANCE: f32 = 1.;

const CANVAS_WIDTH: i32 = 1800;
const CANVAS_HEIGHT: i32 = 800;

fn main() {
    let camera = Vector3::new(-0., 0., 0.);
    let camera_rotation = Rotation3::face_towards(
        &Vector3::new(0., 0., 1.), // direction
        &Vector3::new(0., 1., 0.),   // up
    );

    let mut scene = Scene::new(Vector3::new(0., 0., 0.));
    // red
    scene.objects.push(Object::new(
        //Shape::sphere(Vector3::new(0., -1., 3.), 1.0),
        Shape::Difference(
            Box::new(Shape::sphere(Vector3::new(0., -1., 3.), 1.0)),
            Box::new(Shape::sphere(Vector3::new(0., -0.5, 2.5), 0.5)),
        ),
        Vector3::new(1., 0., 0.),
        Some(500.),
        0.2,
    ));

    // blue
    scene.objects.push(Object::new(
        Shape::sphere(Vector3::new(2., 0., 4.), 1.0),
        Vector3::new(0., 0., 1.),
        Some(500.),
        0.3,
    ));

    // green
    scene.objects.push(Object::new(
        Shape::Intersection(
            Box::new(Shape::sphere(Vector3::new(-2., 0., 4.), 1.0)),
            Box::new(Shape::sphere(Vector3::new(-1., 0., 4.), 1.0)),
        ),
        Vector3::new(0., 1., 0.),
        Some(10.),
        0.4,
    ));

    // yellow
    scene.objects.push(Object::new(
        Shape::sphere(Vector3::new(0., -5001., 0.), 5000.),
        Vector3::new(1., 1., 0.),
        Some(1000.),
        0.5,
    ));

    scene.lights.push(Light::Ambient(0.2));
    scene
        .lights
        .push(Light::Point(0.6, Vector3::new(2., 1., 0.)));
    scene
        .lights
        .push(Light::Directional(0.2, Vector3::new(1., 4., 4.)));

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
            let color = scene.trace_ray(&camera, &direction, 1., f32::MAX, 3);
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

#[derive(Default)]
struct Scene {
    objects: Vec<Object>,
    lights: Vec<Light>,
    background: Vector3<f32>,
}

impl Scene {
    fn new(background: Vector3<f32>) -> Self {
        Self {
            objects: Vec::new(),
            lights: Vec::new(),
            background,
        }
    }

    fn trace_ray(
        &self,
        o: &Vector3<f32>,
        d: &Vector3<f32>,
        t_min: f32,
        t_max: f32,
        depth: u8,
    ) -> Vector3<f32> {
        self.closest_intersection(o, d, t_min, t_max)
            .map(|(object, intersection)| {
                let p = o + intersection.t * d;
                let n = intersection.normal;
                let local_color =
                    object.color * self.compute_lighting(&object.specular, &p, &n, &-d);
                if depth <= 0 || object.reflective <= 0. {
                    local_color
                } else {
                    let r = reflect_ray(&-d, &n);
                    let reflected_color = self.trace_ray(&p, &r, 0.001, f32::INFINITY, depth - 1);
                    // weighted average of local and reflective colors
                    local_color * (1. - object.reflective) + reflected_color * object.reflective
                }
            })
            .unwrap_or(self.background)
    }

    fn compute_lighting(
        &self,
        specular: &Option<f32>,
        p: &Vector3<f32>,
        n: &Vector3<f32>,
        v: &Vector3<f32>,
    ) -> f32 {
        self.lights
            .iter()
            .map(|light| match light {
                Light::Ambient(i) => *i,
                Light::Point(i, position) => {
                    // looks like p is wrong somehow?
                    let l = position - p;
                    self.compute_directional_light(i, specular, p, n, &l, v, 1.)
                }
                Light::Directional(i, direction) => {
                    self.compute_directional_light(i, specular, p, n, direction, v, f32::INFINITY)
                }
            })
            .sum()
    }

    fn closest_intersection(
        &self,
        o: &Vector3<f32>,
        d: &Vector3<f32>,
        t_min: f32,
        t_max: f32,
    ) -> Option<(&Object, Intersection)> {
        self.objects
            .iter()
            .filter_map(|object| {
                //let intersections = sphere.intersect_ray(&camera, &direction);
                let min_t = object
                    .shape
                    .intersect_ray(&o, &d)
                    .map(|(t1, t2)| *t1.min(&t2))
                    .filter(|intersection| intersection.t > t_min && intersection.t < t_max);
                min_t.map(|t| (object, t))
            })
            .fold(None, |acc, (object, intersection)| {
                if let Some((_min_sphere, min_t)) = acc {
                    if intersection.t < min_t.t {
                        Some((object, intersection))
                    } else {
                        acc
                    }
                } else {
                    Some((object, intersection))
                }
            })
    }

    fn compute_directional_light(
        &self,
        i: &f32,
        specular: &Option<f32>,
        p: &Vector3<f32>,
        n: &Vector3<f32>,
        l: &Vector3<f32>,
        v: &Vector3<f32>,
        t_max: f32,
    ) -> f32 {
        // If shadowed from light, no directional light
        if let Some(_) = self.closest_intersection(p, l, 0.001, t_max) {
            0.
        } else {
            let n_dot_l = n.dot(l).max(0.);
            let diffuse = i * n_dot_l / (n.magnitude() * l.magnitude());
            if let Some(specular) = specular {
                let r = reflect_ray(l, n);
                let r_dot_v = r.dot(v).max(0.);
                let specular = i * (r_dot_v / (r.magnitude() * v.magnitude())).powf(*specular);
                specular + diffuse
            } else {
                diffuse
            }
        }
    }
}

fn reflect_ray(r: &Vector3<f32>, n: &Vector3<f32>) -> Vector3<f32> {
    2. * n * n.dot(r) - r
}

struct Object {
    shape: Shape,
    color: Vector3<f32>,
    specular: Option<f32>,
    reflective: f32,
}

enum Shape {
    Sphere { center: Vector3<f32>, radius: f32 },
    Union(Box<Shape>, Box<Shape>),
    Intersection(Box<Shape>, Box<Shape>),
    Difference(Box<Shape>, Box<Shape>),
}

#[derive(Copy, Clone)]
struct Intersection{
    t: f32,
    normal: Vector3<f32>,
}

impl Intersection {
    fn min<'a>(&'a self, other: &'a Self) -> &'a Self {
        if self.t < other.t {
            self
        } else {
            other
        }
    }

    fn max<'a>(&'a self, other: &'a Self) -> &'a Self {
        if self.t > other.t {
            self
        } else {
            other
        }
    }
}

impl Shape {
    fn sphere(center: Vector3<f32>, radius: f32) -> Self {
        Self::Sphere { center, radius }
    }

    fn intersect_ray(&self, o: &Vector3<f32>, d: &Vector3<f32>) -> Option<(Intersection, Intersection)> {
        match self {
            Self::Sphere { center, radius } => {
                let co = o - center;

                let a = d.dot(d);
                let b = 2. * co.dot(d);
                let c = co.dot(&co) - radius * radius;
                solve_quadratic(a, b, c).map(|(t1, t2)| (
                        Intersection{ t: t1, normal: (o + t1 * d) - center},
                        Intersection{ t: t2, normal: (o + t2 * d) - center},
                ))
            }
            Self::Union(a, b) => {
                // TODO handle non-contiguous
                let intersect_a = a.intersect_ray(o, d);
                let intersect_b = b.intersect_ray(o, d);
                if let Some((ta1, ta2)) = intersect_a {
                    if let Some((tb1, tb2)) = intersect_b {
                        let t1 = ta1.min(&ta2).min(&tb1).min(&tb2);
                        let t2 = ta1.max(&ta2).max(&tb1).max(&tb2);
                        Some((*t1, *t2))
                    } else {
                        intersect_a
                    }
                } else {
                    intersect_b
                }
            }
            Self::Intersection(a, b) => {
                // TODO handle non-contiguous
                let intersect_a = a.intersect_ray(o, d);
                let intersect_b = b.intersect_ray(o, d);
                if let Some((ta1, ta2)) = intersect_a {
                    if let Some((tb1, tb2)) = intersect_b {
                        let ta_min = ta1.min(&ta2);
                        let ta_max = ta1.max(&ta2);
                        let tb_min = tb1.min(&tb2);
                        let tb_max = tb1.max(&tb2);
                        Some((*ta_min.max(tb_min), *ta_max.min(tb_max)))
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            Self::Difference(a, b) => {
                // TODO handle encapsulated properly (should return non-contiguous range)
                let intersect_a = a.intersect_ray(o, d);
                let intersect_b = b.intersect_ray(o, d);
                if let Some((ta1, ta2)) = intersect_a {
                    if let Some((mut tb1, mut tb2)) = intersect_b {
                        // Inverted normals for subtracted object
                        tb1.normal = -tb1.normal;
                        tb2.normal = -tb2.normal;
                        let ta_min = ta1.min(&ta2);
                        let ta_max = ta1.max(&ta2);
                        let tb_min = tb1.min(&tb2);
                        let tb_max = tb1.max(&tb2);
                        if ta_min.t < tb_min.t {
                            Some((*ta_min, *tb_min))
                        } else {
                            Some((*tb_max, *ta_max))
                        }
                    } else {
                        intersect_a
                    }
                } else {
                    None
                }
            }
        }
    }
}

impl Object {
    fn new(shape: Shape, color: Vector3<f32>, specular: Option<f32>, reflective: f32) -> Self {
        Self {
            shape,
            color,
            specular,
            reflective,
        }
    }
}

fn solve_quadratic(a: f32, b: f32, c: f32) -> Option<(f32, f32)>  {
    let discriminant = b * b - 4. * a * c;
    let disc_sqrt = discriminant.sqrt();
    if !disc_sqrt.is_nan() {
        let root1 = (-b + disc_sqrt) / (2. * a);
        let root2 = (-b - disc_sqrt) / (2. * a);
        Some((root1, root2))
    } else {
        None
    }
}

enum Light {
    Ambient(f32),
    Point(f32, Vector3<f32>),
    Directional(f32, Vector3<f32>),
}
