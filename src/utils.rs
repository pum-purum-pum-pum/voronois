use na::{Point3};
use kiss3d::light::Light;
use kiss3d::camera::{ArcBall, FirstPerson};
use kiss3d::window::Window;
use kiss3d::event::{Action, Key, WindowEvent};
use crate::BOX_SIZE;

pub struct KissMe {
	pub eye: Point3<f32>,
	pub at: Point3<f32>,
	pub first_person: FirstPerson,
	pub arc_ball: ArcBall,
	pub use_arc_ball: bool,
	pub window: Window
}

impl KissMe {
	pub fn process_events(&mut self) {
		for event in self.window.events().iter() {
	        match event.value {
	            WindowEvent::Key(key, Action::Release, _) => {
	                if key == Key::Numpad1 {
	                    self.use_arc_ball = true
	                } else if key == Key::Numpad2 {
	                    self.use_arc_ball = false
	                }
	            }
	            _ => {}
	        }
	    }		
	}

	pub fn render(&mut self) {
		if self.use_arc_ball {
			self.window.render_with_camera(&mut self.arc_ball);
		} else {
			self.window.render_with_camera(&mut self.first_person);
		}
	}
}

impl Default for KissMe {
	fn default() -> Self {
		let eye = Point3::new(BOX_SIZE as f32 / 2.0, BOX_SIZE as f32 / 2.0, 2.0);
	    let at = Point3::new(BOX_SIZE as f32 / 2.0, BOX_SIZE as f32 / 2.0, 0.0);
        let mut window = Window::new("Kiss3d: lines");
		window.set_point_size(5.0);
	    window.set_light(Light::StickToCamera);
		KissMe {
			eye: eye,
		    at: at,
		    first_person: FirstPerson::new(eye, at),
		    arc_ball: ArcBall::new(eye, at),
		    use_arc_ball: true,
		    window: window
		}
	}
}