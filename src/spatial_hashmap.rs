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
    let x = x.round() as usize;
    let y = y.round() as usize;

    let index = self.index(x, y);
    self.cells[index].push(data);
  }

  pub fn query(&self, x: f32, y: f32) -> Vec<usize> {
    let radius = self.query_radius;

    let left = (x - radius).round().max(0.0) as usize;
    let right = (x + radius).round().min(self.width as f32 - 1.0) as usize;
    let bottom = (y - radius).round().max(0.0) as usize;
    let top = (y + radius).round().min(self.height as f32 - 1.0) as usize;

    let mut results = Vec::<usize>::new();

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

  pub fn clear(&mut self) {
    self.cells.clear()
  }

  fn index(&self, i: usize, j: usize) -> usize {
    let i = i.max(0).min(self.width - 1);
    let j = j.max(0).min(self.height - 1);

    i + j * self.width
  }
}
