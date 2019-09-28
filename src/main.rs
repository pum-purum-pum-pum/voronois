// WIP! THIS IS SLOW DRAFTY IMPLEMENTATION(you can read TODO comments to understand current state) FOR PROCEDURAL DESTRUCTION.

// TODO remove extern
extern crate kiss3d;
extern crate nalgebra as na;
extern crate ncollide2d;
extern crate rand;
extern crate voronoi;
use kiss3d::camera::Camera;
use rand::Rng;
use na::{Point2, Point3};

pub mod utils;
pub mod destruction;
use utils::KissMe;
use destruction::*;

fn main() {
    use kiss3d::event::{Action, Key, WindowEvent};
    let mut kiss_me = KissMe::default();
    let mut colored_segments: Vec<(Vec<(Point2<f32>, Point2<f32>)>, Point3<f32>)> = vec![];
    let mut mouse = Point2::new(0f32, 0f32);
    let mut bounding_poly: Vec<Point2<f32>> = vec![];
    let mut rng = rand::thread_rng();
    while !kiss_me.window.should_close() {
        for event in kiss_me.window.events().iter() {
            match event.value {
                WindowEvent::CursorPos(x, y, _modif) => {
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
                    let (vor_polys, _vor_pts, _clip_points) =
                        destruction(&bounding_poly, Point2::new(mouse.x, mouse.y), 20);
                    for poly in vor_polys.iter() {
                        let current_color = Point3::new(
                            rng.gen_range(0.1, 1.0),
                            rng.gen_range(0.1, 1.0),
                            rng.gen_range(0.1, 1.0),
                        );
                        let cur_segments = to_segments(poly);
                        colored_segments.push((cur_segments, current_color))
                    }
                }
                WindowEvent::Key(key, Action::Release, _) => {
                    match key {
                        Key::Space => {
                            let input_bounding_poly =
                                generate_convex_polygon(10, BOX_SIZE as f32 / 2.0);
                            bounding_poly = vec![];
                            for p in input_bounding_poly.iter() {
                                bounding_poly.push(Point2::new(
                                    p.x + BOX_SIZE as f32 / 2.0,
                                    p.y + BOX_SIZE as f32 / 2.0,
                                ))
                            }
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
        for (segments, color) in colored_segments.iter() {
            for segment in segments.iter() {
                my_draw_line(&mut kiss_me, &segment, &color);
            }
        }
        kiss_me.window.draw_point(
            &Point3::new(mouse.x, mouse.y, 0.0),
            &Point3::new(0.0, 0.0, 0.0),
        );
        kiss_me.render();
    }
}
