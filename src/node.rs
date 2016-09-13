extern crate nalgebra;

use nalgebra::{Vector2};

#[derive(Copy, Clone, Debug)]
pub struct PathData
{
	pub g: f32,
	pub h: f32,
	pub f: f32,
	pub parent: Option<Vector2<i32>>,
}

#[derive(Copy, Clone, Debug)]
pub struct Node
{
	pub pos: Vector2<i32>,
	pub is_obs: bool,
	pub by_obs: u8,
	pub path_data: PathData,
}

impl Node
{
	pub fn new(x: i32, y: i32) -> Node
	{
		Node
		{
			pos: Vector2::new(x, y),
			is_obs: false,
			by_obs: 0,
			path_data: PathData
			{
				g: 0.,
				h: 0.,
				f: 0.,
				parent: None
			}
		}
	}

	pub fn clear_path(&mut self) -> ()
	{
		self.path_data = PathData
		{
			g: 0.,
			h: 0.,
			f: 0.,
			parent: None
		}
	}
}
