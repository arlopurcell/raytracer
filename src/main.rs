use image::{Rgb, RgbImage};
use nalgebra::{geometry::Rotation3, Vector3};
use rayon::prelude::*;
use std::cmp::Ordering;

const VIEWPORT_WIDTH: f32 = 2.;
const VIEWPORT_HEIGHT: f32 = 1.;
const VIEWPORT_DISTANCE: f32 = 1.;

const CANVAS_WIDTH: i32 = 1800;
const CANVAS_HEIGHT: i32 = 800;

fn main() {
    let camera = Vector3::new(0., 0., -1.);
    let camera_rotation = Rotation3::face_towards(
        &Vector3::new(0., 0., 1.), // direction
        &Vector3::new(0., 1., 0.),   // up
    );

    let mut scene = Scene::new(Vector3::new(0., 0., 0.));
    /*
    // red
    scene.objects.push(Object::new(
        //Shape::sphere(Vector3::new(0., -1., 3.), 1.0),
        Shape::Difference(
            Box::new(Shape::sphere(Vector3::new(0., -1., 3.), 1.0)),
            Box::new(Shape::sphere(Vector3::new(0.3, -0.5, 2.5), 0.4)),
        ),
        Vector3::new(1., 0., 0.),
        Some(500.),
        0.2,
        0.,
        1.33,
    ));

    // blue
    scene.objects.push(Object::new(
        Shape::sphere(Vector3::new(2., 0., 4.), 1.0),
        Vector3::new(0., 0., 1.),
        Some(500.),
        0.3,
        0.,
        1.33,
    ));

    // green
    scene.objects.push(Object::new(
        Shape::Intersection(
            Box::new(Shape::sphere(Vector3::new(-2., 0., 4.), 1.0)),
            Box::new(Shape::sphere(Vector3::new(-3., 0., 3.), 1.0)),
        ),
        Vector3::new(0., 1., 0.),
        Some(10.),
        0.2,
        0.,
        1.33,
    ));

    // yellow
    scene.objects.push(Object::new(
        Shape::half_space(Vector3::new(0., 1., 0.), -1.),
        Vector3::new(1., 1., 0.),
        Some(1000.),
        0.5,
        0.,
        1.33,
    ));

    // clear ball
    scene.objects.push(Object::new(
        Shape::sphere(Vector3::new(0.75, -0.5, 1.8), 0.25),
        Vector3::new(1., 1., 1.),
        Some(10.),
        0.0,
        0.8,
        1.33,
    ));

    // clear ball stand
    scene.objects.push(Object::new(
        Shape::sphere(Vector3::new(0.75, -1.75, 1.8), 1.),
        Vector3::new(1., 0.6, 0.),
        Some(10.),
        0.0,
        0.,
        1.33,
    ));

    // mirror ball
    scene.objects.push(Object::new(
        Shape::sphere(Vector3::new(-1.5, 1., 6.), 2.),
        Vector3::new(1., 1., 1.),
        Some(10.),
        0.9,
        0.,
        1.33,
    ));
    */

        /*
    scene.objects.push(Object::new(
        Shape::sphere(Vector3::new(0., 0., 2.), 1.0),
        Vector3::new(0., 0., 1.),
        Some(500.),
        0.3,
        0.,
        1.33,
    ));
    scene.objects.push(Object::new(
        Shape::half_space(Vector3::new(1., 0., 0.).normalize(), 0.5),
        Vector3::new(1., 1., 1.),
        Some(500.),
        0.3,
        0.,
        1.33,
    ));
    */

    // cube
    scene.objects.push(Object::new(
        Shape::sphere(Vector3::new(0., 0., 2.), 9.)
            .difference(Shape::half_space(Vector3::new(1., 0., 0.), -0.5))
            .difference(Shape::half_space(Vector3::new(-1., 0., 0.), -0.5))
            .difference(Shape::half_space(Vector3::new(0., 1., 0.), -0.5))
            .difference(Shape::half_space(Vector3::new(0., -1., 0.), -0.5))
            .difference(Shape::half_space(Vector3::new(0., 0., 1.), 1.5))
            .difference(Shape::half_space(Vector3::new(0., 0., -1.), -2.5))
        ,
        Vector3::new(1., 1., 0.),
        Some(1000.),
        0.5,
        0.,
        1.33,
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
        refraction: f32,
        depth: u8,
    ) -> Vector3<f32> {
        self.closest_intersection(o, d, t_min, t_max)
            .map(|(object, intersection)| {
                let p = o + intersection.t * d;
                let n = intersection.normal;
                let local_color =
                    object.color * self.compute_lighting(&object.specular, &p, &n, &-d);
                if depth <= 0 {
                    local_color
                } else {
                    let r = reflect_ray(&-d, &n);
                    let reflected_term = if object.reflection > 0. {
                        object.reflection * self.trace_ray(&p, &r, 0.001, f32::INFINITY, refraction, depth - 1)
                    } else {
                        Vector3::new(0., 0., 0.)
                    };

                    let refracted_term = if object.transparency > 0. {
                        let r = refraction / object.refraction;
                        let c = (-n).dot(d);
                        let refraction_direction =  r * d + (r * c - (1.- r*r * (1. - c*c))) * n;

                        let refracted_color = self.trace_ray(&p, &refraction_direction, 0.001, f32::INFINITY, object.refraction, depth - 1);
                        refracted_color * object.transparency
                    } else {
                        Vector3::new(0., 0., 0.)
                    };

                    // weighted average of local, reflective, and refractive colors
                    local_color * (1. - object.reflection - object.transparency) + reflected_term + refracted_term
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
                object
                    .shape
                    .intersect_ray(&o, &d, t_min, t_max)
                    .map(|t| (object, t))
            })
            .fold(None, |acc, (object, intersection)| {
                if let Some((_min_object, min_i)) = acc {
                    if intersection.t < min_i.t {
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
        // If shadowed from light, less directional light
        let shadower_transparency = if let Some((object, _intersection)) = self.closest_intersection(p, l, 0.001, t_max) {
            object.transparency
        } else {
            1.
        };
        if shadower_transparency > 0. {
            let n_dot_l = n.dot(l).max(0.);
            let diffuse = i * n_dot_l / (n.magnitude() * l.magnitude());
            let light = if let Some(specular) = specular {
                let r = reflect_ray(l, n);
                let r_dot_v = r.dot(v).max(0.);
                let specular = i * (r_dot_v / (r.magnitude() * v.magnitude())).powf(*specular);
                specular + diffuse
            } else {
                diffuse
            };
            light * shadower_transparency
        } else {
            0.
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
    reflection: f32,
    transparency: f32,
    refraction: f32,
}

enum Shape {
    Sphere { center: Vector3<f32>, radius: f32 },
    HalfSpace { normal: Vector3<f32>, distance: f32 },
    Union(Box<Shape>, Box<Shape>),
    Intersection(Box<Shape>, Box<Shape>),
    Difference(Box<Shape>, Box<Shape>),
}

#[derive(Copy, Clone, PartialEq, Debug)]
struct Intersection{
    t: f32,
    normal: Vector3<f32>,
    entering: bool,
}

impl Intersection {
    fn invert(self) -> Self {
        Intersection {t: self.t, normal: -self.normal, entering: !self.entering}
    }
}

/*
impl Ord for Intersection {
    fn cmp(&self, other: &Self) -> Ordering {
        self.t.partial_cmp(&other.t).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for Intersection {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.t.partial_cmp(&other.t)
    }
}

impl PartialEq for Intersection {
    fn eq(&self, other: &Self) -> bool {
        self.t == other.t
    }
}

impl Eq for Intersection {}
*/

impl Shape {
    fn sphere(center: Vector3<f32>, radius: f32) -> Self {
        Self::Sphere { center, radius }
    }

    fn half_space(normal: Vector3<f32>, distance: f32) -> Self {
        Self::HalfSpace { normal, distance }
    }

    fn union(self, other: Self) -> Self {
        Self::Union(Box::new(self), Box::new(other))
    }

    fn intersection(self, other: Self) -> Self {
        Self::Intersection(Box::new(self), Box::new(other))
    }

    fn difference(self, other: Self) -> Self {
        Self::Difference(Box::new(self), Box::new(other))
    }

    fn intersect_ray(&self, o: &Vector3<f32>, d: &Vector3<f32>, t_min: f32, t_max: f32) -> Option<Intersection> {
        self.intersect_ray_rec(o, d).into_iter().filter(|i| i.t > t_min && i.t < t_max && i.entering).next()
    }

    fn intersect_ray_rec(&self, o: &Vector3<f32>, d: &Vector3<f32>) -> Vec<Intersection> {
        // TODO something weird happens when the camera is inside the shape.
        match self {
            Self::Sphere { center, radius } => {
                let co = o - center;

                let a = d.dot(d);
                let b = 2. * co.dot(d);
                let c = co.dot(&co) - radius * radius;
                if let Some((t1, t2)) = solve_quadratic(a, b, c) {
                    let t_min = t1.min(t2);
                    let t_max = t1.max(t2);
                    vec![
                        Intersection{ t: t_min, normal: (o + t_min * d) - center, entering: true},
                        Intersection{ t: t_max, normal: (o + t_max * d) - center, entering: false},
                    ]
                } else {
                    vec![]
                }
            }
            Self::HalfSpace { normal, distance } => {
                let denom = d.dot(normal);
                if denom != 0. {
                    let t = (distance - o.dot(normal)) / d.dot(normal);
                    if t > 0. {
                        vec![Intersection { t, normal: *normal, entering: true }]
                    } else {
                        vec![]
                    }
                } else {
                    vec![]
                }
            }
            Self::Union(a, b) => {
                let mut intersections_a = a.intersect_ray_rec(o, d).into_iter().peekable();
                let mut intersections_b = b.intersect_ray_rec(o, d).into_iter().peekable();

                let mut result = vec![];
                let mut in_a = false;
                let mut in_b = false;
                loop {
                    let next_a = intersections_a.peek();
                    let next_b = intersections_b.peek();

                    if !next_a.is_some() && !next_b.is_some() {
                        break;
                    }

                    if next_a.is_some() && (!next_b.is_some() || next_a.unwrap().t < next_b.unwrap().t) {
                        let next_a = intersections_a.next().unwrap();
                        if !in_b && in_a  && !next_a.entering {
                            // was only in a, now in neither
                            result.push(next_a);
                        } else if !in_b && !in_a && next_a.entering {
                            // was in neither, now in union
                            result.push(next_a);
                        }
                        in_a = next_a.entering
                    } else {
                        let next_b = intersections_b.next().unwrap();
                        if !in_a && in_b  && !next_b.entering {
                            // was only in b, now in neither
                            result.push(next_b);
                        } else if !in_a && !in_b && next_b.entering {
                            // was in neither, now in union
                            result.push(next_b);
                        }
                        in_b = next_b.entering
                    }
                } 
                result
            }
            Self::Intersection(a, b) => {
                /*
                let mut intersections_a = a.intersect_ray_rec(o, d).into_iter().peekable();
                let mut intersections_b = b.intersect_ray_rec(o, d).into_iter().peekable();
                */
                let mut intersections_a = a.intersect_ray_rec(o, d);
                let mut intersections_b = b.intersect_ray_rec(o, d);

                //dbg!(&intersections_a);
                //dbg!(&intersections_b);

                let mut intersections_a = intersections_a.into_iter().peekable();
                let mut intersections_b = intersections_b.into_iter().peekable();

                let mut result = vec![];
                let mut in_a = false;
                let mut in_b = false;
                loop {
                    let next_a = intersections_a.peek();
                    let next_b = intersections_b.peek();

                    if !next_a.is_some() && !next_b.is_some() {
                        break;
                    }

                    if next_a.is_some() && (!next_b.is_some() || next_a.unwrap().t < next_b.unwrap().t) {
                        let next_a = intersections_a.next().unwrap();
                        if in_a && in_b && !next_a.entering {
                            // was in both, now in just b
                            result.push(next_a);
                        } else if in_b && !in_a && next_a.entering {
                            // was in just b, now in both
                            result.push(next_a)
                        }
                        in_a = next_a.entering
                    } else {
                        let next_b = intersections_b.next().unwrap();
                        if in_b && in_a && !next_b.entering {
                            // was in both, now in just a
                            result.push(next_b);
                        } else if in_a && !in_b && next_b.entering {
                            // was in just a, now in both
                            result.push(next_b)
                        }
                        in_b = next_b.entering
                    }
                } 
                result
            }
            Self::Difference(a, b) => {
                let mut intersections_a = a.intersect_ray_rec(o, d).into_iter().peekable();
                let mut intersections_b = b.intersect_ray_rec(o, d).into_iter().peekable();

                let mut result = vec![];
                let mut in_a = false;
                let mut in_b = false;
                loop {
                    let next_a = intersections_a.peek();
                    let next_b = intersections_b.peek();

                    if !next_a.is_some() && !next_b.is_some() {
                        break;
                    }

                    if next_a.is_some() && (!next_b.is_some() || next_a.unwrap().t < next_b.unwrap().t) {
                        let next_a = intersections_a.next().unwrap();
                        if in_a && !in_b && !next_a.entering {
                            // was in just a, now in neither
                            result.push(next_a);
                        } else if !in_b && !in_a && next_a.entering {
                            // was in neither, now in just a
                            result.push(next_a)
                        }
                        in_a = next_a.entering
                    } else {
                        let next_b = intersections_b.next().unwrap();
                        if in_b && in_a && !next_b.entering {
                            // was in both, now in just a
                            result.push(next_b.invert());
                        } else if in_a && !in_b && next_b.entering {
                            // was in just a, now in both
                            result.push(next_b.invert())
                        }
                        in_b = next_b.entering
                    }
                } 
                result
            }
        }
    }
}


impl Object {
    fn new(shape: Shape, color: Vector3<f32>, specular: Option<f32>, reflection: f32, transparency: f32, refraction: f32) -> Self {
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

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_difference() {
        let death_star = Shape::Difference(
            Box::new(Shape::sphere(Vector3::new(0., 0., 3.), 1.0)),
            Box::new(Shape::sphere(Vector3::new(0., 0., 4.), 0.5)),
        );

        let intersections = death_star.intersect_ray_rec(&Vector3::new(0., 0., 0.), &Vector3::new(0., 0., 1.));
        assert_eq!(
            vec![
                   Intersection{
                       t: 2.,
                       normal: Vector3::new(0., 0., -1.),
                       entering: true,
                   },
                   Intersection{
                       t: 3.5,
                       normal: Vector3::new(0., 0., 0.5),
                       entering: false,
                   },
            ],
            intersections,
        )
    }

    #[test]
    fn test_plane_intersection() {
        let planes = Shape::Intersection(
            Box::new(Shape::half_space(Vector3::new(1., 0., 0.), -1.)),
            Box::new(Shape::half_space(Vector3::new(0., 1., 0.), -1.)),
        );
        let planes = Shape::half_space(Vector3::new(1., 0., 0.), -1.);

        let intersections = planes.intersect_ray_rec(&Vector3::new(0., 0., 0.), &Vector3::new(-1., 0., 1.));
        let expected: Vec<Intersection> = vec![
            Intersection {
                t: 3.,
                normal: Vector3::new(0., 1., 0.),
                entering: true,
            }
        ];
        assert_eq!(
            expected,
            intersections,
        );

        /*
        let intersections = planes.intersect_ray_rec(&Vector3::new(0., 0., 0.), &Vector3::new(-1., 1., 1.));
        let expected: Vec<Intersection> = vec![
            Intersection{
                t: 3.5,
                normal: Vector3::new(0., 0., 0.5),
                entering: false,
            },
        ];
        assert_eq!(
            expected,
            intersections,
        );
        */
    }
}
