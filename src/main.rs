use ductr::AnymapImage;
use rand::Rng;

#[derive(Clone,Debug)]
struct Map {
    height: usize,
    width: usize,
    buffer: Vec<u8>,
}



fn get_within_range<T>(value: T, min: T, max: T) -> T
where
    T: PartialOrd + PartialEq
{

    if value < min {
	min
    }
    else if value > max {
	max
    } else {
	value
    }
	
	
    
}


fn jaccard_index(map1: &Map, map2: &Map, tol: usize) -> Option<f64> {

    let mut intersection: f64 = 0.0;
    let mut union = 0;

    if map1.width != map2.width || map1.height != map2.height {
	return None;
    }

    let bmap1 = map1;
    let bmap2 = map2;



    for i in 0..bmap1.height {
	for j in 0..bmap1.width {
	    if let Some(val1) = bmap1.get(j, i) {
		
		if let Some(val2) = bmap2.get(j, i) {

		    if val1 == 1 || val2 == 1 {
			union += 1;
		    }

		    if val1 == 1 && val2 == 1 {
			intersection += 1.0;
		    } else {
			// create two neighborhood vectors and add it up
			// tolerance
			if tol != 0 {
			    // create two vectors
			    // let tolmap1 = bmap1.lidar_reading_no_rotation(j, i, tol);
			    // let tolmap2 = bmap2.lidar_reading_no_rotation(j, i, tol);

			    // // let tolval = jaccard_index(&tolmap1, &tolmap2, 0).unwrap();
			    // // println!("{i}X{j}: {tolval}");
			    // intersection += tolval;

			}

		    }
		}
	    }
	}
    }

    if union != 0 {
	Some(intersection as f64 / union as f64)
    } else  {
	None
    }

}




impl Map {
    fn new(buffer: Vec<u8>, height: usize, width: usize) -> Self {

	Map {
	    height, width, buffer
	}

    }

    fn get(&self, x: usize, y: usize) -> Option<u8> {

	if x >= self.width || y >= self.height {
	    None
	} else {
	    self.buffer.get(y*self.width + x ).copied()
	}

    }

    fn set(&mut self, x: usize, y: usize, val: u8) -> Option<()> {
	// this is not a nice rust code, but it works so its fine


	if x >= self.width || y >= self.height {
	    return None;
	} else {
	    match self.buffer.get_mut(y*self.width + x ) {
		None => return None,
		Some(v) => {
		    *v = val;
		}
	    }
	}
	
	Some(())
    }



    fn make_black_rectangle(&mut self, x: usize, y: usize, rad: usize) {


	let min_point_x = if rad > x {
	    0
	} else {
	    get_within_range(x - rad, 0, self.width)
	};

	let min_point_y = if rad > y {
	    0
	} else {
	    get_within_range(y - rad, 0, self.width)
	};

	
	let max_point_x = get_within_range(x + rad, 0, self.width);
	let max_point_y = get_within_range(y + rad, 0, self.height);
	

	for i in min_point_y..max_point_y{
	    for j in min_point_x..max_point_x{
		self.set(j, i, 0).unwrap();
	    }
	}


    }

    fn make_robot(&mut self, x: usize, y: usize ) {
	self.make_black_rectangle(x, y, 20);
    }


    fn lidar_reading_no_rotation(&self, x: usize, y: usize, rad: usize) -> Self {

	let min_point_x = if rad > x {
	    0
	} else {
	    get_within_range(x - rad, 0, self.width)
	};

	let min_point_y = if rad > y {
	    0
	} else {
	    get_within_range(y - rad, 0, self.width)
	};


	

	
	let max_point_x = get_within_range(x + rad, 0, self.width);
	let max_point_y = get_within_range(y + rad, 0, self.height);

	let mut vec: Vec<u8> = Vec::new();

	
	for i in min_point_y..max_point_y{
	    for j in min_point_x..max_point_x{

		let data =  self.get(j, i).unwrap();
		vec.push(data);

	    }
	}

	Map {
	    height: max_point_y - min_point_y,
	    width: max_point_x - min_point_x,
	    buffer: vec
	}
    }

    fn to_pgm(&self) -> AnymapImage {

	let mut vectordata: Vec<u8> = Vec::new();
	
	
	for i in 0..self.height{
	    for j in 0..self.width{
		
		vectordata.push(self.get(j, i).unwrap());
		
	    }
	}


	let newimage = AnymapImage::pgm(vectordata, 255,self.height, self.width).unwrap();
	    
	    
	newimage

    }


    
    fn to_pbm(&self) -> AnymapImage {
	
	let mut vectordata: Vec<u8> = Vec::new();
	
	
	for i in 0..self.height{
	    for j in 0..self.width{
		
		vectordata.push(self.get(j, i).unwrap());
		
	    }
	}


	let newimage = AnymapImage::pbm(vectordata, self.height, self.width).unwrap();
	    
	    
	newimage

    }



    fn get_points(&self) -> Self  {
	let buff: Vec<u8> = self.buffer.iter().map(|x| {

	    if *x == 0 as u8 {1}
	    else {0}
	}).collect();


	Map { height: self.height, width: self.width, buffer: buff }
    }

    fn separate_and_set(&mut self, m: usize) {
	let rows = self.height;
	let cols = self.width;
	
	// Iterate over the matrix in m*m squares
	for i in (0..rows).step_by(m) {
            for j in (0..cols).step_by(m) {
		// Check if the current square is within bounds
		if i + m <= rows && j + m <= cols {
                    // Check if the square contains at least one '1'
                    let mut contains_one = false;
                    for x in i..(i + m) {
			for y in j..(j + m) {

//			    println!("{x} x {y}");
			    
                            if self.get(y, x).unwrap() == 1 {
				contains_one = true;
				break;
                            }
			}
			if contains_one {
                            break;
			}
                    }
                    
                    // If the square contains '1', set all elements to '1'
                    if contains_one {
			for x in i..(i + m) {
                            for y in j..(j + m) {
				self.set(y, x, 1);
                            }
			}
                    }
		}
            }
	}
    }
    

    
}

#[derive(Clone)]
struct Particle {
    x: usize,
    y: usize,
}

impl Particle {

    fn random(xmin: usize, xmax: usize, ymin: usize, ymax: usize) -> Self {
	Particle {
	    x: rand::thread_rng().gen_range(xmin..xmax),
	    y: rand::thread_rng().gen_range(ymin..ymax),
	}


    }

    fn moveparticle(&mut self, xa: usize, ya:usize) {
	self.x += xa;
	self.y += ya;
    }


}

fn add_particles(list: &mut Vec<Particle>, xc: usize, yc: usize, rad: usize, number: usize) {
    for i in 0..number {
	list.push(Particle::random(xc - rad, xc+ rad, yc - rad, yc + rad));
    }
}

fn main() {
    println!("Hello, Particle Fileter!");

    let cat = AnymapImage::read_from_binary("bettermap.pgm").expect("couldn't open the binary file");

    let (height, width) = cat.dimensions();

    let mut map_data = Map::new(cat.get_buffer(), height, width);

    let mut robot_location_map = map_data.clone();

    let mut for_lidar_map = map_data.clone();

    let mut another_map =map_data.clone();

    let mut particle_list = Vec::new();

    // initialize random particles 

    add_particles(&mut particle_list, 600, 600, 50, 1000);
    add_particles(&mut particle_list, 650, 650, 100, 1000);
    add_particles(&mut particle_list, 500, 500, 200, 10000);
    add_particles(&mut particle_list, 800, 500, 200, 10000);
    add_particles(&mut particle_list, 500, 800, 200, 10000); 




    particle_list.iter().for_each(|particle| {
	map_data.make_black_rectangle(particle.x, particle.y, 1);
    });
    

    let mut realparticle = Particle {
	x: 500,
	y: 800,
    };
    
    map_data.to_pgm().write_as_binary("particles.pgm").unwrap();

    robot_location_map.make_black_rectangle(realparticle.x, realparticle.y, 4);

    robot_location_map.to_pgm().write_as_binary("robot.pgm").unwrap();

    let mut robotscan = for_lidar_map.lidar_reading_no_rotation(realparticle.x, realparticle.y, 100).get_points();
    robotscan.separate_and_set(7);


    robotscan.to_pbm().write_as_binary("thing.pbm").unwrap();



    particle_list = particle_list.into_iter().filter(|particle|{
	let mut this_scan = for_lidar_map.lidar_reading_no_rotation(particle.x, particle.y, 100).get_points();
	this_scan.separate_and_set(7);

	if let Some(val) = jaccard_index(&robotscan, &this_scan, 0) {
	    if val > 0.5 {true} else {false}
	} else {false}
	
    }).collect();



    particle_list.iter().for_each(|particle| another_map.make_black_rectangle(particle.x, particle.y, 1));
    
    another_map.to_pgm().write_as_binary("another_datalol.pgm").unwrap();


    // move the robot and particle a bit

    realparticle.moveparticle(30, 7);

    particle_list = particle_list.into_iter().map(|mut particle| {
	particle.moveparticle(30, 7);
	particle
    }).collect();

    let len = particle_list.len();

    dbg!(len);

    for i in 0..len {
	let currx = particle_list.get(i).unwrap().x;
	let curry = particle_list.get(i).unwrap().y;
	// Borrow Checker Bullshit. immutable refrence drops here, therby we can now use an mutable refrence
	add_particles(&mut particle_list, currx , curry , 20, 80);
    }

    dbg!(particle_list.len());


    let mut whatever = for_lidar_map.clone();
    let mut anywhere = for_lidar_map.clone();
    let mut shitlol = for_lidar_map.clone();
    
    
    particle_list.iter().for_each(|particle| {
	 whatever.make_black_rectangle(particle.x, particle.y, 1);
    });


    
    whatever.to_pgm().write_as_binary("whatever_particles.pgm").unwrap();


    robotscan = for_lidar_map.lidar_reading_no_rotation(realparticle.x, realparticle.y, 100).get_points();

    robotscan.separate_and_set(7);

    robotscan.to_pbm().write_as_binary("another.pbm").unwrap();

    shitlol.make_robot(realparticle.x, realparticle.y);

    shitlol.to_pgm().write_as_binary("nextrobot_position.pgm").unwrap();

    // now filter the particles


    particle_list = particle_list.into_iter().filter(|particle|{
	let mut this_scan = for_lidar_map.lidar_reading_no_rotation(particle.x, particle.y, 100).get_points();
	this_scan.separate_and_set(7);
	
	if let Some(val) = jaccard_index(&robotscan, &this_scan, 0) {
	    if val > 0.5 {true} else {false}
	} else {false}
	
    }).collect();
    
    
    particle_list.iter().for_each(|particle| anywhere.make_black_rectangle(particle.x, particle.y, 1));
    
    anywhere.to_pgm().write_as_binary("anywhere_datalol.pgm").unwrap();

    dbg!(particle_list.len()) ;

    // i should make it possible to do these things without coding by just inputting things but i am bored, program to change stuff
    
    
}
