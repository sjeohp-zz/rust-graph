use std::vec::Vec;

use node::*;

fn cmp_find(a: &Node, b: &Node) -> bool
{
	a.pos.x == b.pos.x && a.pos.y == b.pos.y
}

fn cmp_insert(a: &Node, b: &Node) -> bool
{
	if a.path_data.f == b.path_data.f {
		return a.path_data.h <= b.path_data.h;
	}
    return a.path_data.f <= b.path_data.f;
}

pub struct Fringe
{
	data: Vec<Node>,
}

impl Fringe
{
	pub fn new() -> Fringe
	{
		Fringe
		{
			data: Vec::new(),
		}
	}

	pub fn insert(&mut self, value: Node) -> bool
	{
		let mut insert_loc: usize = 0;
		for i in 0..self.data.len()
		{
			if cmp_find(&value, &self.data[i])
			{
				return false;
			}
			if cmp_insert(&value, &self.data[i])
			{
				insert_loc = i + 1;
			}
		}
		self.data.insert(insert_loc, value);
		return true;
	}

	pub fn pop(&mut self) -> Option<Node>
	{
		self.data.pop()
	}

	pub fn len(&self) -> usize
	{
		self.data.len()
	}

	pub fn find(&self, value: Node) -> bool
	{
		for i in 0..self.data.len()
		{
			if cmp_find(&value, &self.data[i])
			{
				return true;
			}
		}
		return false;
	}

	pub fn update(&mut self, value: Node) -> bool
	{
		let s = self.data.len();
		let mut curr_loc = s;
		for i in 0..s
		{
			if cmp_find(&value, &self.data[i])
			{
				curr_loc = i;
			}
		}
		for i in curr_loc..s
		{
			if i+1 < s && cmp_insert(&value, &self.data[i+1])
			{
				self.data[i] = self.data[i+1].clone();
			}
			else
			{
				self.data[i] = value;
				return true;
			}
		}
		return false;
	}
}
