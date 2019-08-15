extern crate voronoi;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate rand;
extern crate ncollide2d;
use kiss3d::camera::Camera;

use ncollide2d::transformation::convex_hull_idx;
use rand::Rng;
use voronoi::{voronoi, Point, make_polygons, DCEL};
use na::{Point3, Point2};
use utils::KissMe;

pub const BOX_SIZE: f64 = 1.;
pub const TEMPERATURE: f32 = 6.0; // for more temperature sites sampled closer to bullet position
pub const DISPERSION: f32 = 200.0; // dispersion of sampled voronoi sites 

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

pub fn check_in(polygon: &[Point2<f32>], point: Point2<f32>) -> bool {
	polygon
		.iter()
		.all(|p| p.coords.perp(&point.coords) > 0.0)
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

pub fn destruction(
	polygon: &[Point2<f32>], 
	bullet: Point2<f32>,
	sites_number: usize
) ->  (Vec<Vec<Point2<f32>>>, Vec<Point2<f32>>) {
	// TODO rewrite everything in one iteration closed form if possible
	let mut weights: Vec<f32> = polygon.iter()
		.map(
			|poly_point| {
				(poly_point.coords - bullet.coords).norm()
			}
		)
		.collect();
	// oh...need to be more carefull with overflow here
	{	// apply sofmax
		let temperatire = TEMPERATURE; // actually 1/sigma
		let exp_sum: f32 = weights.iter().map(|x| (-temperatire * x).exp()).sum();
		for x in weights.iter_mut() {*x = (-temperatire * *x).exp() / exp_sum};
	}
	let mut sites = vec![];
	for _ in 0..sites_number {
		sites.push(convert(spawn_inside(polygon, &weights, DISPERSION)));
	}

	let vor_diagram = voronoi(sites.clone(), BOX_SIZE);
	let vor_polys = my_make_polygons(&vor_diagram);
	(
		vor_polys, 
		sites
			.iter()
			.map(|p| Point2::new(*p.x as f32, *p.y as f32))
			.collect()
	)
}

fn my_make_polygons(vor_diagram: &DCEL) -> Vec<Vec<Point2<f32>>> {
	let vor_polys = make_polygons(&vor_diagram);
	let mut polys = vec![];
	for poly in vor_polys.iter() {
		let poly: Vec<Point2<f32>> = poly.iter().map(
			|p| {
				Point2::new(*p.x as f32, *p.y as f32)
			}
		).collect();
		polys.push(poly);
	}
	polys
}

fn to_segments(poly: &[Point2<f32>]) -> Vec<(Point2<f32>, Point2<f32>)> {
	assert!(poly.len() > 0);
	let mut res = vec![];
	let mut shift_poly = poly.iter();
	shift_poly.next();
	for (a, b) in poly.iter().zip(shift_poly) {
		res.push((
			*a,
			*b
		))
	}
	res.push((*poly.first().unwrap(), *poly.last().unwrap()));
	res
}


fn proj0(point: Point2<f32>) -> Point3<f32> {
	Point3::new(point.x, point.y, 0.0)
}

fn main() {
    use kiss3d::event::{Action, Key, WindowEvent};
    let mut kiss_me = KissMe::default();
    let mut red_segments: Vec<(Point2<f32>, Point2<f32>)> = vec![];
    let mut grey_segments: Vec<(Point2<f32>, Point2<f32>)> = vec![];
    let mut white_dots: Vec<Point2<f32>> = vec![];
    let mut mouse = Point2::new(0f32, 0f32);
    let mut bounding_poly: Vec<Point2<f32>> = vec![];
	while !kiss_me.window.should_close()  {
		for event in kiss_me.window.events().iter() {
	        match event.value {
	        	WindowEvent::CursorPos(x, y, _modif) => {
    			    red_segments = vec![];
				    white_dots = vec![];
                    let last_pos = na::Point2::new(x as f32, y as f32);
				    let window_size =
	                        na::Vector2::new(kiss_me.window.size()[0] as f32, kiss_me.window.size()[1] as f32);
	                let mouse_pair = kiss_me.arc_ball.unproject(&last_pos, &window_size);
                    let mouse3d = mouse_pair.0 - mouse_pair.0.z * mouse_pair.1 / mouse_pair.1.z;
                    mouse = Point2::new(mouse3d.x, mouse3d.y);
                    if bounding_poly.len() == 0 {continue};
				    let (vor_polys, vor_pts) = destruction(
				    	&bounding_poly, 
				    	Point2::new(mouse.x, mouse.y), 
				    	20
				    );
				    for poly in vor_polys.iter() {
				    	let cur_segments = to_segments(poly);
						red_segments.extend(cur_segments.iter());
				    }
				    white_dots.extend(vor_pts.iter());

                }
	            WindowEvent::Key(key, Action::Release, _) => {
	            	match key {
	            		Key::Space => {
	        			    red_segments = vec![];
						    white_dots = vec![];
						    grey_segments = vec![];
						    let input_bounding_poly = generate_convex_polygon(10, BOX_SIZE as f32 / 2.0);
						    bounding_poly = vec![];
						    for p in input_bounding_poly.iter() {
						    	bounding_poly.push(Point2::new(p.x + BOX_SIZE as f32 / 2.0, p.y + BOX_SIZE as f32 / 2.0))
						    }
						    grey_segments.extend(to_segments(&bounding_poly));
						    // let poly_segments = 
	            		}
	            		_ => ()
	            	}
	            }
	            _ => ()
	        }
	    }

        // update the current camera.
        kiss_me.process_events();
        // draw everything
		for segment in red_segments.iter() {
			kiss_me.window.draw_line(&proj0(segment.0), &proj0(segment.1), &Point3::new(1.0, 0.0, 0.0));
		}
		for segment in grey_segments.iter() {
			kiss_me.window.draw_line(&proj0(segment.0), &proj0(segment.1), &Point3::new(0.5, 0.5, 0.5));			
		}
		for point in white_dots.iter() {
			kiss_me.window.draw_point(&proj0(*point), &Point3::new(1.0, 1.0, 1.0));
		}
		kiss_me.window.draw_point(&Point3::new(mouse.x, mouse.y, 0.0), &Point3::new(0.0, 1.0, 0.0));
		kiss_me.render();
	}
}
