#![feature(box_syntax)]

extern crate nalgebra;

use nalgebra::{Vector2};

use std::cmp;
use std::f32::consts;
use std::collections::{VecDeque};

mod node;
use node::{Node, PathData};
mod fringe;
use fringe::{Fringe};

pub const ROWS: usize = 1024;
pub const COLS: usize = 1024;

const ALPHA: f32 = consts::SQRT_2;
const DIMEN: f32 = 1.;

fn delta(from: Vector2<i32>, to: Vector2<i32>) -> Vector2<i32>
{
	return Vector2::new( (to.x - from.x).abs(), (to.y - from.y).abs() );
}

fn calc_h(delta: Vector2<i32>) -> f32
{
	let d: f32 = 1000.;
	let a: f32 = (d * ALPHA).round() / d;
	return DIMEN * ((delta.x + delta.y) as f32 - (2. - a) * (cmp::min(delta.x, delta.y) as f32));
}

fn graph_from_fract(fract_pos: Vector2<f32>) -> Vector2<i32>
{
	Vector2::new(
		(fract_pos.x * (ROWS as f32)) as i32,
		(fract_pos.y * (COLS as f32)) as i32)
}

fn fract_from_graph(graph_pos: Vector2<i32>) -> Vector2<f32>
{
	Vector2::new(
		graph_pos.x as f32 / (ROWS as f32),
		graph_pos.y as f32 / (COLS as f32))
}

pub struct Graph
{
	tax: f32,
	pub nodes: Box<[[Node; COLS]; ROWS]>,
}

impl Graph
{
	pub fn new() -> Graph
	{
		let mut graph = Graph
		{
			tax: 0.01,
			nodes: box [[Node::new(0, 0); COLS]; ROWS]
		};
		for x in 0..ROWS
		{
			for y in 0..COLS
			{
				graph.nodes[x][y] = Node::new(x as i32, y as i32);
			}
		}
		graph
	}

	fn node_at(&self, pos: Vector2<i32>) -> &Node
	{
		assert!(self.contains_node_at(pos), "tried to access node outside bounds");
		return &self.nodes[pos.x as usize][pos.y as usize];
	}

	fn node_at_mut(&mut self, pos: Vector2<i32>) -> &mut Node
	{
		assert!(self.contains_node_at(pos), "tried to access node outside bounds");
		return &mut self.nodes[pos.x as usize][pos.y as usize];
	}

	fn contains_node_at(&self, pos: Vector2<i32>) -> bool
	{
		pos.x >= 0 &&
		pos.x < ROWS as i32 &&
		pos.y >= 0 &&
		pos.y < COLS as i32
	}

	fn neighbours_octal_at(&self, pos: Vector2<i32>) -> [Option<Vector2<i32>>; 8]
	{
		let mut neighbours: [Option<Vector2<i32>>; 8] = [None; 8];
		let mut i: usize = 0;
		for dx in -1..2
		{
			for dy in -1..2
			{
				let npos = Vector2::new(pos.x + dx, pos.y + dy);
				if npos != pos && self.contains_node_at(npos)
				{
					neighbours[i] = Some(npos);
					i += 1;
				}
			}
		}
		neighbours
	}

	fn rem_obs_at(&mut self, pos: Vector2<i32>) -> ()
	{
		self.node_at_mut(pos).is_obs = false;
		let neighbours = self.neighbours_octal_at(pos);
		for i in 0..8
		{
			match neighbours[i]
			{
				Some(pos) =>
				{
					let node = self.node_at_mut(pos);
					if node.by_obs > 0 { node.by_obs -= 1; }
				},
				_ => ()
			}
		}
	}

	fn add_obs_at(&mut self, pos: Vector2<i32>) -> ()
	{
		self.node_at_mut(pos).is_obs = true;
		let neighbours = self.neighbours_octal_at(pos);
		for i in 0..8
		{
			match neighbours[i]
			{
				Some(pos) =>
				{
					self.node_at_mut(pos).by_obs += 1;
				},
				_ => ()
			}
		}
	}

	pub fn add_obs_square(&mut self, origin: Vector2<f32>, other: Vector2<f32>)
	{
		let origin = graph_from_fract(origin);
		let other = graph_from_fract(other);
		println!("obstacle graph: {} {} ==> {} {}", origin.x, origin.y, other.x, other.y);
		for x in origin.x..other.x+1
		{
			for y in origin.y..other.y+1
			{
				self.add_obs_at(Vector2 { x: x, y: y});
			}
		}
	}

	fn los_bresenham(&self, from: Vector2<i32>, to: Vector2<i32>) -> bool
	{
		let mut x0 = from.x;
		let mut y0 = from.y;
		let x1 = to.x;
		let y1 = to.y;

		let dx = ((x1 - x0) as i32).abs();
		let sx = if x0 < x1 { 1 } else { -1 };
		let dy = ((y1 - y0) as i32).abs();
		let sy = if y0 < y1 { 1 } else { -1 };
		let mut err = (if dx > dy { dx } else { -dy })/2;
		let mut e2: i32;

		loop
		{
			if self.node_at(Vector2::new(x0, y0)).is_obs
			{
				return false;
			}
			if x0 == x1 && y0 == y1
			{
				return true;
			}
			e2 = err;
			if e2 > -dx
			{
				err -= dy;
				x0 += sx;
			}
			if e2 < dy
			{
				err += dx;
				y0 += sy;
			}
		}
	}

	fn path_grandparent(&mut self, curr: Vector2<i32>) -> Option<Vector2<i32>>
	{
		match self.node_at_mut(curr).path_data.parent
		{
			Some(curr_parent) =>
			{
				match self.node_at_mut(curr_parent).path_data.parent
				{
					Some(curr_parent_parent) => { Some(curr_parent_parent) },
					None => { None }
				}
			},
			None => { None }
		}
	}

	fn path_smooth_back(&mut self, end: Vector2<i32>) -> ()
	{
		let mut curr = end;
		let mut next_opt = self.path_grandparent(curr);
		loop
		{
			match next_opt
			{
				Some(next) =>
				{
					if self.los_bresenham(curr, next)
					{
						self.node_at_mut(curr).path_data.parent = next_opt;
						next_opt = match self.node_at_mut(next).path_data.parent
						{
							Some(next_parent) => { Some(next_parent) },
							None => { None }
						};
					}
					else
					{
						curr = match self.node_at_mut(curr).path_data.parent
						{
							Some(curr_parent) => { curr_parent },
							None => { break }
						};
						next_opt = self.path_grandparent(curr);
					}
				},
				None => { break; }
			}
		}
	}

	fn path_data_for(&self, curr: Vector2<i32>, goal: Vector2<i32>, parent: Option<Vector2<i32>>) -> PathData
	{
		let mut g = 0.;
		let h;
		match parent
		{
			Some(prev) =>
			{
				let delta_prev = delta(prev, curr);
				let delta_goal = delta(curr, goal);
				g = self.node_at(prev).path_data.g + calc_h(delta_prev);
				h = calc_h(delta_goal);

			},
			None =>
			{
				let delta_goal = delta(curr, goal);
				h = calc_h(delta_goal);
			}
		}
		return PathData
		{
			g: g,
			h: h,
			f: g + h,
			parent: parent,
		}
	}

	fn path_comp_parent(&self, curr: Vector2<i32>, next: Vector2<i32>) -> Option<Vector2<i32>>
	{
		let curr_node = self.node_at(curr);
		let next_node = self.node_at(next);

		match curr_node.path_data.parent
		{
			Some(prev) =>
			{
				let delta = delta(prev, next);
				let prev_node = self.node_at(prev);
				if 	prev_node.by_obs == 0 &&
					curr_node.by_obs == 0 &&
					next_node.by_obs == 0 &&
					self.los_bresenham(prev_node.pos, next_node.pos)
				{
					match next_node.path_data.parent
					{
						Some(_) =>
						{
							if prev_node.path_data.g + calc_h(delta) < next_node.path_data.g
							{
								return Some(prev);
							}
							return None;
						},
						None => { return Some(prev); }
					}
				}
				else
				{
					match next_node.path_data.parent
					{
						Some(_) =>
						{
							if curr_node.path_data.g + calc_h(delta) < next_node.path_data.g
							{
								return Some(curr);
							}
							return None;
						},
						None => { return Some(curr); }
					}
				}
			},
			None => { return Some(curr); }
		}
	}

	fn path_update_fringe(&mut self, goal: Vector2<i32>, mut fringe: &mut Fringe, closed: &mut Vec<Vector2<i32>>) -> bool
	{
		match fringe.pop()
		{
			Some(curr_node) =>
			{
				// println!("fringe pop {} {}", curr_node.pos.x, curr_node.pos.y);
				closed.push(curr_node.pos);
				if curr_node.pos == goal
				{
					return true;
				}
				if self.los_bresenham(curr_node.pos, goal)
				{
					closed.push(goal);
					self.node_at_mut(goal).path_data.parent = Some(curr_node.pos);
					return true;
				}
				let neighbours = self.neighbours_octal_at(curr_node.pos);
				for i in 0..8
				{
					match neighbours[i]
					{
						Some(neighbour_pos) =>
						{
							// println!("neighbour at {} {}", neighbour_pos.x, neighbour_pos.y);
							if 	!self.node_at(neighbour_pos).is_obs &&
								!closed.contains(&neighbour_pos)
							{
								match self.path_comp_parent(curr_node.pos, neighbour_pos)
								{
									Some(parent_pos) =>
									{
										self.node_at_mut(neighbour_pos).path_data = self.path_data_for(neighbour_pos, goal, Some(parent_pos));
										if self.node_at_mut(neighbour_pos).by_obs > 0
										{
											self.node_at_mut(neighbour_pos).path_data.f += self.tax;
										}
										if fringe.find(self.node_at_mut(neighbour_pos).clone())
										{
											// println!("updating node in fringe");
											fringe.update(self.node_at_mut(neighbour_pos).clone());
										}
										else
										{
											// println!("inserting node in fringe");
											fringe.insert(self.node_at_mut(neighbour_pos).clone());
										}
									}
									None => {}
								}
							}
						},
						None => {}
					}
				}
			},
			None => {}
		}
		return false;
	}

	fn nearest_non_obs_to(&self, pos: Vector2<i32>) -> Option<Vector2<i32>>
	{
		if !self.node_at(pos).is_obs { return Some(pos); }

		let mut queue: VecDeque<Vector2<i32>> = VecDeque::new();
		queue.push_back(pos);

		while queue.len() > 0
		{
			match queue.pop_front()
			{
				Some(curr_pos) =>
				{
					if !self.node_at(curr_pos).is_obs { return Some(curr_pos); }
					let neighbours = self.neighbours_octal_at(curr_pos);
					for i in 0..8
					{
						match neighbours[i]
						{
							Some(neighbour_pos) =>
							{
								let mut include = true;
								for v in queue.iter()
								{
									if v.x == neighbour_pos.x && v.y == neighbour_pos.y
									{
										include = false;
									}
								}
								if include
								{
									queue.push_back(neighbour_pos);
								}
							},
							None => {}
						}
					}
				},
				None => { println!("--Odd, shouldn't be here")}
			}
		}
		return None;
	}

	pub fn path_compute(&mut self, start: Vector2<f32>, goal: Vector2<f32>) -> Option<Vec<Vector2<f32>>>
	{
		let start = graph_from_fract(start);
		let mut goal = graph_from_fract(goal);

		if self.node_at(goal).is_obs
		{
			match self.nearest_non_obs_to(goal)
			{
				Some(new_goal) =>
				{
					goal = new_goal;
				},
				None => { println!("--No path (all nodes are landscape)"); return None; }
			}
		}

		let mut fringe = Fringe::new();
		let mut closed: Vec<Vector2<i32>> = Vec::new();
		self.node_at_mut(start).path_data = self.path_data_for(start, goal, None);
		fringe.insert(self.node_at(start).clone());
		while !self.path_update_fringe(goal, &mut fringe, &mut closed)
		{
			if fringe.len() == 0
			{
				println!("--No path (no route to goal)");
				return None;
			}
		}

		self.path_smooth_back(goal);
		let mut opt = Some(goal);
		let mut path_fract_rv: Vec<Vector2<f32>> = Vec::new();
		loop
		{
			match opt
			{
				Some(p) =>
				{
					path_fract_rv.push(fract_from_graph(p));
					opt = self.node_at(p).path_data.parent;
					self.node_at_mut(p).clear_path();
				},
				None => { break; }
			}
		}

		for i in 0..closed.len()
		{
			self.node_at_mut(closed[i]).clear_path();
		}

		Some(path_fract_rv)
	}

	pub fn path_compute_debug(&mut self, start: Vector2<f32>, goal: Vector2<f32>) -> Option<(Vec<Vector2<f32>>, Vec<Vector2<f32>>)>
	{
		let start = graph_from_fract(start);
		let mut goal = graph_from_fract(goal);

		println!("computing path from {} {} to {} {}", start.x, start.y, goal.x, goal.y);

		if self.node_at(goal).is_obs
		{
			match self.nearest_non_obs_to(goal)
			{
				Some(new_goal) =>
				{
					goal = new_goal;
				},
				None => { println!("--No path (all nodes are landscape)"); return None; }
			}
		}

		let mut fringe = Fringe::new();
		let mut closed: Vec<Vector2<i32>> = Vec::new();
		self.node_at_mut(start).path_data = self.path_data_for(start, goal, None);
		fringe.insert(self.node_at(start).clone());
		while !self.path_update_fringe(goal, &mut fringe, &mut closed)
		{
			if fringe.len() == 0
			{
				println!("--No path (no route to goal)");
				return None;
			}
		}

		self.path_smooth_back(goal);
		let mut opt = Some(goal);
		let mut path_fract_rv: Vec<Vector2<f32>> = Vec::new();
		loop
		{
			match opt
			{
				Some(p) =>
				{
					path_fract_rv.push(fract_from_graph(p));
					opt = self.node_at(p).path_data.parent;
					self.node_at_mut(p).clear_path();
				},
				None => { break; }
			}
		}

		let mut closed_fract: Vec<Vector2<f32>> = Vec::with_capacity(closed.len());
		for i in 0..closed.len()
		{
			closed_fract.push(fract_from_graph(closed[i]));
			self.node_at_mut(closed[i]).clear_path();
		}

		Some((path_fract_rv, closed_fract))
	}
}
