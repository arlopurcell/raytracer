use nalgebra::{geometry::{Rotation3, Translation3, Transform3}, Vector3, Matrix4, Point3, Unit};

#[derive(Debug, PartialEq)]
pub enum Shape {
    Sphere { center: Point3<f32>, radius: f32 },
    HalfSpace { normal: Unit<Vector3<f32>>, distance: f32 },
    Quadric{ matrix: Matrix4<f32>, normal_matrix: Matrix4<f32> },
    Union(Box<Shape>, Box<Shape>),
    Intersection(Box<Shape>, Box<Shape>),
    Difference(Box<Shape>, Box<Shape>),
}

impl Shape {
    pub fn sphere(center: Point3<f32>, radius: f32) -> Self {
        Self::Sphere { center, radius }
        /*
        Self::quadric(Matrix4::new(
                1., 0., 0., 0.,
                0., 1., 0., 0.,
                0., 0., 1., 0.,
                0., 0., 0., -(radius * radius),
        )).translate(&center.coords)
        */
    }

    pub fn quadric(matrix: Matrix4<f32>) -> Self {
        let normal_matrix = Matrix4::from_fn(|i, j| matrix[(i, j)] + matrix[(j, i)]);
        Self::Quadric{matrix, normal_matrix}
    }

    pub fn half_space(normal: Vector3<f32>, distance: f32) -> Self {
        Self::HalfSpace { normal: Unit::new_normalize(normal), distance }
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

    fn normal(&self, p: &Point3<f32>) -> Option<Unit<Vector3<f32>>> {
        match self {
            Self::Sphere { center, radius: _ } => Some(Unit::new_normalize(p - center)),
            Self::HalfSpace { normal, distance: _} => Some(*normal),
            Self::Quadric{ matrix: _, normal_matrix } => {
                let v = normal_matrix * p.to_homogeneous();
                Some(Unit::new_normalize(Vector3::new(v.x, v.y, v.z)))
                //Vector3::from_homogeneous()
            }
            /*
            Self::Quadric(a) => {
                Some(Vector3::new(
                        p.x * 2. * a[(0,0)] + p.y * (a[(0,1)] + a[(1,0)]) + p.z * (a[(0,2)] + a[(2,0)]) + a[(0, 3)] + a[(3, 0)],
                        p.x * (a[(0,1)] + a[(1,0)]) + p.y * 2. * a[(1,1)] + p.z * (a[(2,1)] + a[(1,2)]) + a[(1, 3)] + a[(3,1)],
                        p.x * (a[(0,2)] + a[(2,0)]) + p.y * (a[(2,1)] + a[(1,2)]) + p.z * 2. * a[(2,2)] + a[(2, 3)] + a[(3,2)],
                ).normalize())
            }
            */
            Self::Union(a, b) => a.normal(p).or_else(|| b.normal(p)),
            Self::Intersection(a, b) => a.normal(p).or_else(|| b.normal(p)),
            Self::Difference(a, b) => a.normal(p).or_else(|| b.normal(p).map(|n| -n)),
        }
    }

    pub fn translate(self, translation: &Vector3<f32>) -> Self {
        match self {
            Self::Sphere { center, radius } => Self::Sphere { center: center + translation, radius },
            Self::HalfSpace { normal, distance } => Self::HalfSpace { normal, distance: distance + translation.dot(&normal) },
            Self::Quadric{ matrix, normal_matrix: _} => {
                let translation: Translation3<f32> = (-*translation).into();
                let transform: Transform3<f32> = nalgebra::convert(translation);
                let transform_matrix: Matrix4<f32> = transform.into_inner();
                Self::quadric(transform_matrix.transpose() * matrix * transform_matrix)
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
            Self::Quadric{matrix, normal_matrix: _} => {
                let transform: Transform3<f32> = nalgebra::convert(*rotation);
                let transform_matrix: Matrix4<f32> = transform.into_inner();
                Self::quadric(transform_matrix.transpose() * matrix * transform_matrix)
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

    pub fn intersect_ray(&self, o: &Point3<f32>, d: &Vector3<f32>, t_min: f32, t_max: f32) -> Option<Intersection> {
        self.intersect_ray_rec(o, d).into_iter().filter(|i| i.t > t_min && i.t < t_max && i.entering).next()
    }

    fn intersect_ray_rec(&self, o: &Point3<f32>, d: &Vector3<f32>) -> Vec<Intersection> {
        match self {
            Self::Sphere { center, radius } => {
                let co = o - center;

                let a = d.dot(d);
                let b = 2. * co.dot(d);
                let c = co.dot(&co) - radius * radius;
                if let Some((t1, t2)) = solve_quadratic(a, b, c) {
                    let t_min = t1.min(t2);
                    let t_max = t1.max(t2);

                    let point_min = o + t_min * d;
                    let point_max = o + t_max * d;
                    vec![
                        Intersection{ t: t_min, normal: self.normal(&point_min).unwrap(), entering: true},
                        Intersection{ t: t_max, normal: self.normal(&point_max).unwrap(), entering: false},
                    ]
                } else {
                    vec![]
                }
            }
            Self::HalfSpace { normal, distance } => {
                let denom = d.dot(normal);
                if denom != 0. {
                    let t = (distance - o.coords.dot(normal)) / denom;
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
            Self::Quadric{matrix, normal_matrix: _} => {
                let d = d.to_homogeneous();
                let o = o.to_homogeneous();

                let qd = matrix * d;
                let qo = matrix * o;
                let a = d.dot(&qd);
                let b = o.dot(&qd) + d.dot(&qo);
                let c = o.dot(&qo);
                if let Some((t1, t2)) = solve_quadratic(a, b, c) {
                    let t_min = t1.min(t2);
                    let t_max = t1.max(t2);

                    let point_min = Point3::from_homogeneous(o + t_min * d).unwrap();
                    let point_max = Point3::from_homogeneous(o + t_max * d).unwrap();
                    vec![
                        Intersection{ t: t_min, normal: self.normal(&point_min).unwrap(), entering: true},
                        Intersection{ t: t_max, normal: self.normal(&point_max).unwrap(), entering: false},
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
    pub normal: Unit<Vector3<f32>>,
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

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_quadric_sphere_origin() {
        let quadric = Shape::quadric(Matrix4::new(
                1., 0., 0., 0.,
                0., 1., 0., 0.,
                0., 0., 1., 0.,
                0., 0., 0., -1.,
        ));
        let sphere = Shape::Sphere{center: Point3::new(0., 0., 0.), radius: 1.};

        let camera = Point3::new(0., 0., -3.);
        let d = Vector3::new(0., 0., 1.);

        let quadric_intersections = quadric.intersect_ray_rec(&camera, &d);
        assert_eq!(quadric_intersections.len(), 2);
        let sphere_intersections = sphere.intersect_ray_rec(&camera, &d);
        assert_eq!(quadric_intersections, sphere_intersections);
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
        let sphere = Shape::Sphere{center: Point3::new(0., 0., 0.), radius: 1.};

        let camera = Point3::new(0., 0., -2.);
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
        //assert_eq!(Point3::new(1., 1., -1.).to_homogeneous(), transform_matrix * Point3::new(0., 0., 0.).to_homogeneous());
        //assert_eq!(Point3::new(3., 4., 3.).to_homogeneous(), transform_matrix * Point3::new(2., 3., 4.).to_homogeneous());

        let quadric = Shape::quadric(Matrix4::new(
            1., 0., 0., 0.,
            0., 1., 0., 0.,
            0., 0., 1., 0.,
            0., 0., 0., -1.,
        ));
        let transformed_quadric = Shape::quadric(Matrix4::new(
            1., 0., 0., -1.,
            0., 1., 0., -1.,
            0., 0., 1., 1.,
            -1., -1., 1., 2.,
        ));
        assert_eq!(transformed_quadric, quadric.translate(&translation));
    }
}

