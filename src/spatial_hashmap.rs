pub struct SpatialHashMap {
  width: usize,
  height: usize,
  width_real: f32,
  height_real: f32,
  query_radius: f32,
  cells: Vec<Vec<usize>>,
}

impl SpatialHashMap {
  pub fn new(
    width_real: f32,
    height_real: f32,
    query_radius: f32,
    cell_size: f32,
  ) -> SpatialHashMap {
    let width = (width_real / cell_size).ceil() as usize;
    let height = (height_real / cell_size).ceil() as usize;

    SpatialHashMap {
      width,
      height,
      width_real,
      height_real,
      query_radius,
      cells: vec![Vec::new(); width * height],
    }
  }

  pub fn insert(&mut self, x: f32, y: f32, data: usize) {
    // println!("{}", data);
    // print!("({},{})", x, y);

    let (x, y) = self.normalize_position(x, y);
    // print!(" -> ({},{})", x, y);
    let (i, j) = (x.floor() as usize, y.floor() as usize);
    // println!(" -> ({},{})", i, j);

    let index = self.index(i, j);
    self.cells[index].push(data);
  }

  pub fn query(&self, x: f32, y: f32) -> Vec<usize> {
    let x = x.max((-self.width_real) / 2.0).min(self.width_real / 2.0);
    let y = y.max((-self.height_real) / 2.0).min(self.height_real / 2.0);

    let (x, y) = self.normalize_position(x, y);

    let (left, right, bottom, top) = self.get_neighbor_indices(x, y);

    let mut results = Vec::<usize>::new();
    // println!(
    //   "left: {}, right:{}, bottom:{}, top:{}",
    //   left, right, bottom, top
    // );

    for i in left..=right {
      for j in bottom..=top {
        let index = self.index(i, j);

        for value in &self.cells[index] {
          results.push(value.clone());
        }
      }
    }

    results
  }

  fn get_neighbor_indices(&self, x: f32, y: f32) -> (usize, usize, usize, usize) {
    let radius = self.query_radius;

    let left = (x - radius).round().max(0.0) as usize;
    let right = (x + radius).round().min(self.width as f32 - 1.0) as usize;
    let bottom = (y - radius).round().max(0.0) as usize;
    let top = (y + radius).round().min(self.height as f32 - 1.0) as usize;

    (left, right, bottom, top)
  }

  pub fn clear(&mut self) {
    for cell in &mut self.cells {
      cell.clear();
    }
  }

  fn index(&self, i: usize, j: usize) -> usize {
    let i = i.max(0).min(self.width - 1);
    let j = j.max(0).min(self.height - 1);

    i + j * self.width
  }

  fn normalize_position(&self, x: f32, y: f32) -> (f32, f32) {
    let x = ((x + self.width_real / 2.0) / self.width_real) * self.width as f32;
    let y = ((y + self.width_real / 2.0) / self.width_real) * self.height as f32;

    (x.round(), y.round())
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_dimensions_real() {
    let hashmap = SpatialHashMap::new(40.0, 40.0, 2.0, 10.0);

    assert_eq!(hashmap.width_real, 40.0);
    assert_eq!(hashmap.height_real, 40.0);
    assert_eq!(hashmap.width, 4);
    assert_eq!(hashmap.height, 4);
  }

  #[test]
  fn test_normalize_position() {
    let hashmap = SpatialHashMap::new(40.0, 40.0, 2.0, 10.0);

    let (x, y) = hashmap.normalize_position(0.0, 0.0);
    assert_eq!(x, 2.0);
    assert_eq!(y, 2.0);

    let (x, y) = hashmap.normalize_position(-20.0, -20.0);
    assert_eq!(x, 0.0);
    assert_eq!(y, 0.0);

    let (x, y) = hashmap.normalize_position(20.0, 20.0);
    assert_eq!(x, 4.0);
    assert_eq!(y, 4.0);
  }

  #[test]
  fn test_get_neighbor_indices() {
    let hashmap = SpatialHashMap::new(40.0, 40.0, 1.5, 10.0);

    let (left, right, bottom, top) = hashmap.get_neighbor_indices(2.5, 2.5);
    assert_eq!(left, 1);
    assert_eq!(right, 3);
    assert_eq!(bottom, 1);
    assert_eq!(top, 3);
  }

  #[test]
  fn test_query() {
    let mut hashmap = SpatialHashMap::new(40.0, 40.0, 1.5, 10.0);
    hashmap.insert(-15.0, -15.0, 0);
    hashmap.insert(-5.0, -15.0, 1);
    hashmap.insert(5.0, -15.0, 2);
    hashmap.insert(15.0, -15.0, 3);

    hashmap.insert(-15.0, -5.0, 4);
    hashmap.insert(-5.0, -5.0, 5);
    hashmap.insert(5.0, -5.0, 6);
    hashmap.insert(15.0, -5.0, 7);

    hashmap.insert(-15.0, 5.0, 8);
    hashmap.insert(-5.0, 5.0, 9);
    hashmap.insert(5.0, 5.0, 10);
    hashmap.insert(15.0, 5.0, 11);

    hashmap.insert(-15.0, 15.0, 12);
    hashmap.insert(-5.0, 15.0, 13);
    hashmap.insert(5.0, 15.0, 14);
    hashmap.insert(15.0, 15.0, 15);

    let result = hashmap.query(5.0, 5.0);
    assert_eq!(result, [5, 6, 7, 9, 10, 11, 13, 14, 15].to_vec());
  }

}
