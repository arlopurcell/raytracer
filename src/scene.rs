use nalgebra::{geometry::Rotation3, Vector3};

use crate::object::{Object, Light};
use crate::shape::{Intersection, Shape};

#[derive(Default)]
pub struct Scene {
    pub objects: Vec<Object>,
    pub lights: Vec<Light>,
    pub background: Vector3<f32>,
}

impl Scene {
    pub fn new(background: Vector3<f32>) -> Self {
        Self {
            objects: Vec::new(),
            lights: Vec::new(),
            background,
        }
    }

    pub fn sample_scene_1() -> Self {
        let mut scene = Scene::new(Vector3::new(0., 0., 0.));

        // red
        scene.objects.push(Object::new(
                Shape::sphere(Vector3::new(0., -1., 3.), 1.0)
                .difference(Shape::sphere(Vector3::new(0.3, -0.5, 2.5), 0.4)),
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

        // cube
        scene.objects.push(Object::new(
                Shape::cube(1.)
                .rotate_around_origin(&Rotation3::from_euler_angles(0.5, 0.5, 0.5))
                .translate(&Vector3::new(2., 1.7, 4.))
                ,
                Vector3::new(0., 1., 1.),
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

        scene
    }

    pub fn quadrics() -> Self {
        let mut scene = Scene::new(Vector3::new(0., 0., 0.));

        // red
        scene.objects.push(Object::new(
                Shape::sphere(Vector3::new(0., -1., 3.), 1.0)
                .difference(Shape::sphere(Vector3::new(0.3, -0.5, 2.5), 0.4)),
                Vector3::new(1., 0., 0.),
                Some(500.),
                0.2,
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

        scene
    }

    pub fn trace_ray(
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
