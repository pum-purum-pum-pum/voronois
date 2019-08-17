// WIP! THIS IS SLOW DRAFTY IMPLEMENTATION(you can read TODO comments to understand current state) FOR PROCEDURAL DESTRUCTION.

// TODO remove extern
extern crate kiss3d;
extern crate nalgebra as na;
extern crate ncollide2d;
extern crate rand;
extern crate voronoi;
use kiss3d::camera::Camera;

use na::{Isometry2, Point2, Point3};
use ncollide2d::transformation::convex_hull_idx;
use ncollide2d::{
    query::{Ray, RayCast},
    shape::Segment,
};
use rand::Rng;
use utils::KissMe;
use voronoi::{make_polygons, voronoi, Point, DCEL};

pub const BOX_SIZE: f64 = 1.;
pub const TEMPERATURE: f32 = 6.0; // for more temperature sites sampled closer to bullet position
pub const DISPERSION: f32 = 200.0; // dispersion of sampled voronoi sites
pub const EPS: f32 = 0.00001;
pub const PROX_EPS: f32 = 0.001;

pub mod utils;

pub fn generate_convex_polygon(samples_num: usize, size: f32) -> Vec<Point2<f32>> {
    let mut rng = rand::thread_rng();
    let mut points = vec![];
    for _ in 0..samples_num {
        let x = rng.gen_range(-size, size);
        // sample from circle
        let chord = (size * size - x * x).sqrt();
        let y = rng.gen_range(-chord, chord);
        points.push(Point2::new(x, y));
    }
    let ids = convex_hull_idx(&points);
    // TODO opt: inplace
    let points = {
        let mut res = vec![];
        for &i in ids.iter() {
            res.push(points[i])
        }
        res
    };
    points
}

// check point is inside convex polygon
pub fn check_in(polygon: &[Point2<f32>], point: Point2<f32>) -> bool {
    let mut plus_check_sum = 0usize;
    let mut minus_check_sum = 0usize;
    let poly_segments = to_segments(polygon);
    for segment in poly_segments.iter() {
        let inner_vec = point.coords - segment.0.coords;
        let side_vec = segment.1.coords - segment.0.coords;
        if inner_vec.perp(&side_vec) > -EPS {
            // it's float's ">="
            plus_check_sum += 1;
        }
        if inner_vec.perp(&side_vec) < EPS {
            minus_check_sum += 1;
        }
    }
    minus_check_sum == polygon.len() || plus_check_sum == polygon.len()
}

// check point is inside convex polygon
pub fn check_on(polygon: &[Point2<f32>], point: Point2<f32>) -> bool {
    let poly_segments = to_segments(polygon);
    let mut flag = false;
    for segment in poly_segments.iter() {
        let inner_vec = point.coords - segment.0.coords;
        let side_vec = segment.1.coords - segment.0.coords;
        if inner_vec.perp(&side_vec).abs() < EPS {
            // it's float's ">="
            flag = true;
        }
    }
    flag
}

pub fn spawn_inside(polygon: &[Point2<f32>], weights: &[f32], dispersion: f32) -> Point2<f32> {
    assert!(polygon.len() == weights.len());
    let mut rng = rand::thread_rng();
    let mut res = Point2::new(0.0, 0.0);
    let mut distro_sum = 0f32; // carefull with overflow
    for (point, weight) in polygon.iter().zip(weights.iter()) {
        let final_weight = weight * rng.gen_range(0.0, dispersion);
        res += final_weight * point.coords;
        distro_sum += final_weight;
    }
    let res = res / distro_sum;
    res
}

fn convert(from: Point2<f32>) -> Point {
    Point::new(from.x.into(), from.y.into())
}

fn check(a: f32, b: f32, x: f32) -> bool {
    x - a > -EPS && b - x > -EPS
}

fn check_in_segment(point: Point2<f32>, segment: Segment<f32>) -> bool {
    let min_x = segment.a().x.min(segment.b().x);
    let max_x = segment.a().x.max(segment.b().x);
    let min_y = segment.a().y.min(segment.b().y);
    let max_y = segment.a().y.max(segment.b().y);
    check(min_x, max_x, point.x) && check(min_y, max_y, point.y)
}

// have not found proper function in ncollide
fn segment_intersection(s1: &Segment<f32>, s2: &Segment<f32>) -> Option<Point2<f32>> {
    let dir = s1.b().coords - s1.a().coords;
    let dir2 = s2.b().coords - s2.a().coords;
    // assert!(dir.norm() > EPS && dir2.norm() > EPS);
    if dir.norm() < PROX_EPS || dir2.norm() < PROX_EPS {
        return None;
    }
    let ray1 = Ray::new(*s1.a(), dir);
    let ray2 = Ray::new(*s1.b(), -dir);
    let toi1 = s2.toi_with_ray(&Isometry2::identity(), &ray1, true);
    let toi2 = s2.toi_with_ray(&Isometry2::identity(), &ray2, true);
    if let (Some(toi1), Some(toi2)) = (toi1, toi2) {
        assert!(((s1.a() + toi1 * dir).coords - (s1.b() - toi2 * dir).coords).norm() < PROX_EPS);
        let res = s1.a() + toi1 * dir;
        if check_in_segment(res, s1.clone()) && check_in_segment(res, s2.clone()) {
            Some(res)
        } else {
            None
        }
    } else {
        None
    }
}

// TODO: rewrite this brute force
fn clip_poly(
    polygon: &[Point2<f32>],
    voronoi_poly: &Vec<Point2<f32>>,
) -> (Vec<Point2<f32>>, Vec<Point2<f32>>) {
    let polygon_segments = to_segments(polygon);
    let vor_segments = to_segments(voronoi_poly);
    let mut intersections = vec![];
    for vor_segment in vor_segments.iter() {
        for (_id, poly_segment) in polygon_segments.iter().enumerate() {
            let vor_segment = Segment::new(vor_segment.0, vor_segment.1);
            let poly_segment = Segment::new(poly_segment.0, poly_segment.1);
            if let Some(intersection) = segment_intersection(&vor_segment, &poly_segment) {
                if !check_on(polygon, intersection) {
                    dbg!(intersection);
                    dbg!(vor_segment);
                    dbg!(poly_segment);
                    panic!();
                }
                intersections.push(intersection);
            }
        }
    }
    if intersections.len() > 0 {
        let mut clipped_points = vec![];
        for point in polygon.iter() {
            if check_in(voronoi_poly, *point) {
                clipped_points.push(*point);
            }
        }
        for point in voronoi_poly.iter() {
            if check_in(polygon, *point) {
                clipped_points.push(*point);
            }
        }
        clipped_points.extend(intersections.iter());

        let ids = convex_hull_idx(&clipped_points);
        // TODO opt: inplace
        let clipped_points = {
            let mut res = vec![];
            for &i in ids.iter() {
                res.push(clipped_points[i])
            }
            res
        };

        return (clipped_points, intersections);
    }
    (voronoi_poly.clone(), vec![])
}

fn get_center(points: &[Point2<f32>]) -> Point2<f32> {
    let w = 1.0 / (points.len() as f32);
    let mut center = Point2::new(0f32, 0f32);
    for p in points.iter() {
        center.x += w * p.x;
        center.y += w * p.y;
    }
    center
}

// TODO: rewrite this BRUTE FORCE.
fn brute_clipping(
    polygon: &[Point2<f32>],
    voronois: Vec<Vec<Point2<f32>>>,
) -> (Vec<Vec<Point2<f32>>>, Vec<Point2<f32>>) {
    let mut res = vec![];
    let mut points = vec![];
    for vor_poly in voronois.iter() {
        let (clipped, clip_points) = clip_poly(polygon, vor_poly);
        {
            // TODO: not sure but looks like internal bug in voronoi crate so we need to remove extra cells :O
            if clipped.len() == 0 || !clipped.iter().all(|p| check_in(&polygon, *p)) {
                continue;
            }
        }
        res.push(clipped);
        points.extend(clip_points);
    }
    (res, points)
}

pub fn destruction(
    polygon: &[Point2<f32>],
    bullet: Point2<f32>,
    sites_number: usize,
) -> (Vec<Vec<Point2<f32>>>, Vec<Point2<f32>>, Vec<Point2<f32>>) {
    // TODO rewrite everything in one iteration closed form if possible
    let mut weights: Vec<f32> = polygon
        .iter()
        .map(|poly_point| (poly_point.coords - bullet.coords).norm())
        .collect();
    // oh...need to be more carefull with overflow here mb sort
    {
        // apply sofmax
        let temperatire = TEMPERATURE; // actually 1/sigma
        let exp_sum: f32 = weights.iter().map(|x| (-temperatire * x).exp()).sum();
        for x in weights.iter_mut() {
            *x = (-temperatire * *x).exp() / exp_sum
        }
    }
    let mut sites = vec![];
    for _ in 0..sites_number {
        sites.push(convert(spawn_inside(polygon, &weights, DISPERSION)));
    }

    let vor_diagram = voronoi(sites.clone(), BOX_SIZE);
    let vor_polys = my_make_polygons(&vor_diagram);
    let (vor_polys, clip_points) = brute_clipping(&polygon, vor_polys);
    (
        vor_polys,
        sites
            .iter()
            .map(|p| Point2::new(*p.x as f32, *p.y as f32))
            .collect(),
        clip_points,
    )
}

fn my_make_polygons(vor_diagram: &DCEL) -> Vec<Vec<Point2<f32>>> {
    let vor_polys = make_polygons(&vor_diagram);
    let mut polys = vec![];
    for poly in vor_polys.iter() {
        let poly: Vec<Point2<f32>> = poly
            .iter()
            .map(|p| Point2::new(*p.x as f32, *p.y as f32))
            .collect();
        polys.push(poly);
    }
    polys
}

fn to_segments(poly: &[Point2<f32>]) -> Vec<(Point2<f32>, Point2<f32>)> {
    if poly.len() == 0 {
        return vec![];
    }
    let mut res = vec![];
    let mut shift_poly = poly.iter();
    shift_poly.next();
    for (a, b) in poly.iter().zip(shift_poly) {
        res.push((*a, *b))
    }
    res.push((*poly.last().unwrap(), *poly.first().unwrap()));
    res
}

fn proj0(point: Point2<f32>) -> Point3<f32> {
    proj_z(point, 0.0)
}

fn proj_z(point: Point2<f32>, z: f32) -> Point3<f32> {
    Point3::new(point.x, point.y, z)
}

fn main() {
    use kiss3d::event::{Action, Key, WindowEvent};
    let mut kiss_me = KissMe::default();
    let mut red_segments: Vec<(Point2<f32>, Point2<f32>)> = vec![];
    let mut grey_segments: Vec<(Point2<f32>, Point2<f32>)> = vec![];
    let mut colored_segments: Vec<(Vec<(Point2<f32>, Point2<f32>)>, Point3<f32>)> = vec![];
    let mut white_dots: Vec<Point2<f32>> = vec![];
    let mut blue_dots: Vec<Point2<f32>> = vec![];
    let mut mouse = Point2::new(0f32, 0f32);
    let mut bounding_poly: Vec<Point2<f32>> = vec![];
    let mut rng = rand::thread_rng();
    while !kiss_me.window.should_close() {
        for event in kiss_me.window.events().iter() {
            match event.value {
                WindowEvent::CursorPos(x, y, _modif) => {
                    red_segments = vec![];
                    white_dots = vec![];
                    blue_dots = vec![];
                    colored_segments = vec![];
                    let last_pos = na::Point2::new(x as f32, y as f32);
                    let window_size = na::Vector2::new(
                        kiss_me.window.size()[0] as f32,
                        kiss_me.window.size()[1] as f32,
                    );
                    let mouse_pair = kiss_me.arc_ball.unproject(&last_pos, &window_size);
                    let mouse3d = mouse_pair.0 - mouse_pair.0.z * mouse_pair.1 / mouse_pair.1.z;
                    mouse = Point2::new(mouse3d.x, mouse3d.y);
                    if bounding_poly.len() == 0 {
                        continue;
                    };
                    let (vor_polys, vor_pts, clip_points) =
                        destruction(&bounding_poly, Point2::new(mouse.x, mouse.y), 20);
                    blue_dots.extend(clip_points);
                    for poly in vor_polys.iter() {
                        let current_color = Point3::new(
                            rng.gen_range(0.1, 1.0),
                            rng.gen_range(0.1, 1.0),
                            rng.gen_range(0.1, 1.0),
                        );
                        let cur_segments = to_segments(poly);
                        colored_segments.push((cur_segments, current_color))
                    }
                    white_dots.extend(vor_pts.iter());
                }
                WindowEvent::Key(key, Action::Release, _) => {
                    match key {
                        Key::Space => {
                            red_segments = vec![];
                            white_dots = vec![];
                            grey_segments = vec![];
                            let input_bounding_poly =
                                generate_convex_polygon(10, BOX_SIZE as f32 / 2.0);
                            bounding_poly = vec![];
                            for p in input_bounding_poly.iter() {
                                bounding_poly.push(Point2::new(
                                    p.x + BOX_SIZE as f32 / 2.0,
                                    p.y + BOX_SIZE as f32 / 2.0,
                                ))
                            }
                            grey_segments.extend(to_segments(&bounding_poly));
                            // let poly_segments =
                        }
                        _ => (),
                    }
                }
                _ => (),
            }
        }

        // update the current camera.
        kiss_me.process_events();
        // draw everything
        fn my_draw_line(
            kiss_me: &mut KissMe,
            segment: &(Point2<f32>, Point2<f32>),
            color: &Point3<f32>,
        ) {
            let gran = 200;
            for step in 1..gran {
                let alpha = step as f32 * 1.0 / (gran as f32);
                let cur_point = segment.0 * alpha + segment.1.coords * (1.0 - alpha);
                kiss_me.window.draw_point(&proj0(cur_point), color)
            }
        }
        // for segment in red_segments.iter() {
        // 	// kiss_me.window.draw_line(&proj0(segment.0), &proj0(segment.1), &Point3::new(1.0, 0.0, 0.0));
        // 	my_draw_line(&mut kiss_me, &segment, &Point3::new(1.0, 0.0, 0.0));
        // }
        // for segment in grey_segments.iter() {
        // 	kiss_me.window.draw_line(&proj_z(segment.0, 0.1), &proj_z(segment.1, 0.1), &Point3::new(0.5, 0.5, 0.5));
        // 	// kiss_me.window.add_rectangle((segment.0.coords - segment.1.coords).norm(), 2.0);
        // 	// kiss_me.window.add_quad(segment.0.x, segment.0.y, segment.1.x, segment.1.y);
        // }
        for (segments, color) in colored_segments.iter() {
            for segment in segments.iter() {
                my_draw_line(&mut kiss_me, &segment, &color);
                // kiss_me.window.draw_line(&proj0(segment.0), &proj0(segment.1), &color);
                // kiss_me.window.draw_line(&proj0(segment.0), &proj0(segment.1), &color);
            }
        }
        // for point in white_dots.iter() {
        // 	kiss_me.window.draw_point(&proj0(*point), &Point3::new(1.0, 1.0, 1.0));
        // }

        // for point in blue_dots.iter() {
        // 	kiss_me.window.draw_point(&proj0(*point), &Point3::new(0.0, 0.0, 1.0));
        // }
        kiss_me.window.draw_point(
            &Point3::new(mouse.x, mouse.y, 0.0),
            &Point3::new(0.0, 0.0, 0.0),
        );
        kiss_me.render();
    }
}
