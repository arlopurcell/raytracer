use nalgebra::{geometry::Rotation3, Vector3, Matrix4, Translation3, Transform3, Point3, Vector4};

#[derive(Debug, PartialEq)]
pub enum Shape {
    Sphere { center: Vector3<f32>, radius: f32 },
    HalfSpace { normal: Vector3<f32>, distance: f32 },
    Quadric(Matrix4<f32>),
    Union(Box<Shape>, Box<Shape>),
    Intersection(Box<Shape>, Box<Shape>),
    Difference(Box<Shape>, Box<Shape>),
}

impl Shape {
    pub fn sphere(center: Vector3<f32>, radius: f32) -> Self {
        Self::Quadric(Matrix4::new(
                1., 0., 0., 0.,
                0., 1., 0., 0.,
                0., 0., 1., 0.,
                0., 0., 0., -(radius * radius),
        )).translate(&center)
    }

    pub fn inf_cylinder(radius: f32) -> Self {
        Self::Quadric(Matrix4::new(
                1., 0., 0., 0.,
                0., 0., 0., 0.,
                0., 0., 1., 0.,
                0., 0., 0., -(radius * radius),
        ))
    }

    pub fn cylinder(radius: f32, height: f32) -> Self {
        Self::inf_cylinder(radius)
            .intersection(Self::half_space(Vector3::new(0., 1., 0.), height/2.))
            .intersection(Self::half_space(Vector3::new(0., -1., 0.), height/2.))
    }

    pub fn inf_cone(slope: f32) -> Self {
        Self::Quadric(Matrix4::new(
                1., 0., 0., 0.,
                0., -slope, 0., 0.,
                0., 0., 1., 0.,
                0., 0., 0., 0.,
        ))
    }

    pub fn cone(slope: f32, height: f32) -> Self {
        Self::inf_cone(slope)
            .intersection(Self::half_space(Vector3::new(0., 1., 0.), height))
            .intersection(Self::half_space(Vector3::new(0., -1., 0.), 0.))
    }

    pub fn half_space(normal: Vector3<f32>, distance: f32) -> Self {
        Self::HalfSpace { normal, distance }
    }

    pub fn rectangular_prism(width: f32, height: f32, depth: f32) -> Self {
        Shape::half_space(Vector3::new(0., 0., -1.), 0.5 * depth)
            .intersection(Shape::half_space(Vector3::new(0., 0., 1.), 0.5 * depth))
            .intersection(Shape::half_space(Vector3::new(1., 0., 0.), 0.5 * width))
            .intersection(Shape::half_space(Vector3::new(-1., 0., 0.), 0.5 * width))
            .intersection(Shape::half_space(Vector3::new(0., 1., 0.), 0.5 * height))
            .intersection(Shape::half_space(Vector3::new(0., -1., 0.), 0.5 * height))
    }

    pub fn cube(side_length: f32) -> Self {
        Self::rectangular_prism(side_length, side_length, side_length)
    }

    pub fn union(self, other: Self) -> Self {
        Self::Union(Box::new(self), Box::new(other))
    }

    pub fn intersection(self, other: Self) -> Self {
        Self::Intersection(Box::new(self), Box::new(other))
    }

    pub fn difference(self, other: Self) -> Self {
        Self::Difference(Box::new(self), Box::new(other))
    }

    pub fn translate(self, translation: &Vector3<f32>) -> Self {
        match self {
            Self::Sphere { center, radius } => Self::Sphere { center: center + translation, radius },
            Self::HalfSpace { normal, distance } => Self::HalfSpace { normal, distance: distance + translation.dot(&normal) },
            Self::Quadric(matrix) => {
                let translation: Translation3<f32> = (-*translation).into();
                let transform: Transform3<f32> = nalgebra::convert(translation);
                let transform_matrix: Matrix4<f32> = transform.into_inner();
                Self::Quadric(transform_matrix.transpose() * matrix * transform_matrix)
            }
            Self::Union(mut a, mut b) => {
                *a = a.translate(translation);
                *b = b.translate(translation);
                Self::Union(a, b)
            },
            Self::Intersection(mut a, mut b) => {
                *a = a.translate(translation);
                *b = b.translate(translation);
                Self::Intersection(a, b)
            },
            Self::Difference(mut a, mut b) => {
                *a = a.translate(translation);
                *b = b.translate(translation);
                Self::Difference(a, b)
            },
        }
    }

    pub fn rotate_around_origin(self, rotation: &Rotation3<f32>) -> Self {
        match self {
            Self::Sphere { center, radius } => Self::Sphere { center, radius },
            Self::HalfSpace { normal, distance } => Self::HalfSpace { normal: rotation * normal, distance },
            Self::Quadric(matrix) => {
                let transform: Transform3<f32> = nalgebra::convert(*rotation);
                let transform_matrix: Matrix4<f32> = transform.into_inner();
                Self::Quadric(transform_matrix.transpose() * matrix * transform_matrix)
            }
            Self::Union(mut a, mut b) => {
                *a = a.rotate_around_origin(rotation);
                *b = b.rotate_around_origin(rotation);
                Self::Union(a, b)
            }
            Self::Intersection(mut a, mut b) => {
                *a = a.rotate_around_origin(rotation);
                *b = b.rotate_around_origin(rotation);
                Self::Intersection(a, b)
            }
            Self::Difference(mut a, mut b) => {
                *a = a.rotate_around_origin(rotation);
                *b = b.rotate_around_origin(rotation);
                Self::Difference(a, b)
            }
        }
    }

    pub fn rotate_around_point(self, rotation: &Rotation3<f32>, point: &Vector3<f32>) -> Self {
        self
            .translate(&-point)
            .rotate_around_origin(rotation)
            .translate(point)
    }

    pub fn intersect_ray(&self, o: &Vector3<f32>, d: &Vector3<f32>, t_min: f32, t_max: f32) -> Option<Intersection> {
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
                        Intersection{ t: t_min, normal: ((o + t_min * d) - center).normalize(), entering: true},
                        Intersection{ t: t_max, normal: ((o + t_max * d) - center).normalize(), entering: false},
                    ]
                } else {
                    vec![]
                }
            }
            Self::HalfSpace { normal, distance } => {
                let denom = d.dot(normal);
                if denom != 0. {
                    let t = (distance - o.dot(normal)) / denom;
                    if denom < 0. {
                        vec![
                            Intersection { t, normal: *normal, entering: true },
                        ]
                    } else {
                        vec![
                            Intersection { t: -f32::INFINITY, normal: *normal, entering: true },
                            Intersection { t, normal: *normal, entering: false },
                        ]
                    }
                } else {
                    vec![]
                }
            }
            Self::Quadric(matrix) => {
                let d = d.to_homogeneous();
                let o = Point3::new(o.x, o.y, o.z).to_homogeneous();

                let qd = matrix * d;
                let qo = matrix * o;
                let a = d.dot(&qd);
                let b = o.dot(&qd) + d.dot(&qo);
                let c = o.dot(&qo);
                if let Some((t1, t2)) = solve_quadratic(a, b, c) {
                    let t_min = t1.min(t2);
                    let t_max = t1.max(t2);

                    let point_min = o + t_min * d;
                    let point_max = o + t_max * d;
                    vec![
                        Intersection{ t: t_min, normal: quadric_normal(&point_min, matrix), entering: true},
                        Intersection{ t: t_max, normal: quadric_normal(&point_max, matrix), entering: false},
                    ]
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

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Intersection{
    pub t: f32,
    pub normal: Vector3<f32>,
    pub entering: bool,
}

impl Intersection {
    fn invert(self) -> Self {
        Intersection {t: self.t, normal: -self.normal, entering: !self.entering}
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

fn quadric_normal(point: &Vector4<f32>, matrix: &Matrix4<f32>) -> Vector3<f32> {
    let x = (matrix[(0,0)] + matrix[(0,0)]) * point.x + (matrix[(0,1)] + matrix[(1,0)]) * point.y + (matrix[(0,2)] + matrix[(2,0)]) * point.z + matrix[(0,3)] + matrix[(3,0)];
    let y = (matrix[(1,0)] + matrix[(0,1)]) * point.x + (matrix[(1,1)] + matrix[(1,1)]) * point.y + (matrix[(1,2)] + matrix[(2,1)]) * point.z + matrix[(1,3)] + matrix[(3,1)];
    let z = (matrix[(2,0)] + matrix[(0,2)]) * point.x + (matrix[(2,1)] + matrix[(1,2)]) * point.y + (matrix[(2,2)] + matrix[(2,2)]) * point.z + matrix[(2,3)] + matrix[(3,2)];
    Vector3::new(x, y, z).normalize()
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_quadric_sphere_origin() {
        let quadric = Shape::Quadric(Matrix4::new(
                1., 0., 0., 0.,
                0., 1., 0., 0.,
                0., 0., 1., 0.,
                0., 0., 0., -1.,
        ));
        let sphere = Shape::Sphere{center: Vector3::new(0., 0., 0.), radius: 1.};

        let camera = Vector3::new(0., 0., -3.);
        let d = Vector3::new(0., 0., 1.);

        let quadric_intersections = quadric.intersect_ray_rec(&camera, &d);
        assert_eq!(quadric_intersections.len(), 2);
        let sphere_intersections = sphere.intersect_ray_rec(&camera, &d);
        assert_eq!(quadric_intersections, sphere_intersections);
    }

    #[test]
    fn test_matrix_indexing() {
        let m = Matrix4::new(
            1, 2, 3, 4,
            5, 6, 7, 8,
            9, 10, 11, 12,
            13, 14, 15, 16,
        );

        assert_eq!(1, m[(0,0)]);
        assert_eq!(6, m[(1,1)]);
        assert_eq!(16, m[(3,3)]);

        assert_eq!(2, m[(0,1)]);
        assert_eq!(10, m[(2,1)]);

    }

    /*
    #[test]
    fn test_quadric_sphere_origin_angled() {
        let quadric = Shape::Quadric(Matrix4::new(
                1., 0., 0., 0.,
                0., 1., 0., 0.,
                0., 0., 1., 0.,
                0., 0., 0., -1.,
        ));
        let sphere = Shape::Sphere{center: Vector3::new(0., 0., 0.), radius: 1.};
        let camera = Vector3::new(0., 0., -2.);
        let d = Vector3::new(0.2, 0.2, 1.);
        let quadric_intersections = quadric.intersect_ray_rec(&camera, &d);
        assert_eq!(quadric_intersections.len(), 2);
        let sphere_intersections = sphere.intersect_ray_rec(&camera, &d);
        assert_eq!(quadric_intersections, sphere_intersections);
    }
    */

    #[test]
    fn test_quadric_translation_identity() {
        let translation = Vector3::new(0., 0., 0.);
        let translation: Translation3<f32> = translation.into();
        let transform: Transform3<f32> = nalgebra::convert(translation);
        let transform_matrix: Matrix4<f32> = transform.into_inner();
        assert_eq!(transform_matrix, 
                   Matrix4::new(
                       1., 0., 0., 0.,
                       0., 1., 0., 0.,
                       0., 0., 1., 0.,
                       0., 0., 0., 1.,
                    ));

        let matrix = Matrix4::new(
            1., 0., 0., 0.,
            0., 1., 0., 0.,
            0., 0., 1., 0.,
            0., 0., 0., -1.,
        );
        assert_eq!(matrix, transform_matrix.transpose() * matrix * transform_matrix);
    }

    #[test]
    fn test_quadric_translation() {
        let translation = Vector3::new(1., 1., -1.);
        //assert_eq!(Vector3::new(1., 1., -1.).to_homogeneous(), transform_matrix * Vector3::new(0., 0., 0.).to_homogeneous());
        //assert_eq!(Vector3::new(3., 4., 3.).to_homogeneous(), transform_matrix * Vector3::new(2., 3., 4.).to_homogeneous());

        let quadric = Shape::Quadric(Matrix4::new(
            1., 0., 0., 0.,
            0., 1., 0., 0.,
            0., 0., 1., 0.,
            0., 0., 0., -1.,
        ));
        let transformed_quadric = Shape::Quadric(Matrix4::new(
            1., 0., 0., -1.,
            0., 1., 0., -1.,
            0., 0., 1., 1.,
            -1., -1., 1., 2.,
        ));
        assert_eq!(transformed_quadric, quadric.translate(&translation));
    }

    #[test]
    fn test_quadric_normal() {
        let quadric_ds = Shape::sphere(Vector3::new(0., -1., 3.), 1.0)
                .difference(Shape::sphere(Vector3::new(0.3, -0.5, 2.5), 0.4));

        let sphere_ds = Shape::Sphere{center: Vector3::new(0., -1., 3.), radius: 1.}
                .difference(Shape::Sphere{center: Vector3::new(0.3, -0.5, 2.5), radius: 0.4});

        let camera = Vector3::new(0., 0., 0.);
        for i in -10..10 {
            for j in -20..0 {
                let d = Vector3::new(i as f32 / 10., j as f32 / 10., 1.);

                let quadric_intersections = quadric_ds.intersect_ray_rec(&camera, &d);
                let sphere_intersections = sphere_ds.intersect_ray_rec(&camera, &d);
                for (qi, si) in quadric_intersections.into_iter().zip(sphere_intersections) {
                    assert!((qi.t - si.t).abs() < 0.001);
                    assert!((qi.normal - si.normal).magnitude() < 0.01);
                    assert_eq!(qi.entering, si.entering);
                }
            }
        }
    }
}
