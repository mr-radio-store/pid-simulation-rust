use plotters::prelude::*;

pub struct PidController {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    prev_error: f32,
    min_output: f32,
    max_output: f32,
}

impl PidController {
    pub fn new(kp: f32, ki: f32, kd: f32, min_output: f32, max_output: f32) -> Self {
        Self { kp, ki, kd, integral: 0.0, prev_error: 0.0, min_output, max_output }
    }

    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        if dt <= 0.0 {
            return self.prev_error.clamp(self.min_output, self.max_output);
        }
        let error = setpoint - measurement;
        let p_term = self.kp * error;
        self.integral = (self.integral + error * dt).clamp(self.min_output, self.max_output);
        let i_term = self.ki * self.integral;
        let derivative = (error - self.prev_error) / dt;
        let d_term = self.kd * derivative;

        self.prev_error = error;
        (p_term + i_term + d_term).clamp(self.min_output, self.max_output)
    }
}

pub struct DcMotorSimulation {
    pub speed: f32,
    inertia_inv: f32,
    friction_coeff: f32,
}

impl DcMotorSimulation {
    pub fn new(inertia: f32, friction: f32) -> Self {
        Self { speed: 0.0, inertia_inv: 1.0 / inertia, friction_coeff: friction }
    }

    pub fn step(&mut self, control_torque: f32, load_disturbance: f32, dt: f32) -> f32 {
        let friction_torque = self.speed * self.friction_coeff;
        let net_torque = (control_torque + load_disturbance) - friction_torque;
        let acceleration = net_torque * self.inertia_inv;
        self.speed += acceleration * dt;
        if self.speed < 0.0 { self.speed = 0.0; }
        self.speed
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut pid = PidController::new(3.5, 1.8, 0.12, 0.0, 255.0);
    let mut motor = DcMotorSimulation::new(1.5, 0.25);

    let target_setpoint = 100.0;
    let dt = 0.05;
    let mut external_load = 0.0;

    // Vectors to hold our plotted coordinates
    let mut time_data = Vec::new();
    let mut speed_data = Vec::new();

    // 1. Run the simulation loop and collect metrics
    for tick in 0..200 {
        let time = tick as f32 * dt;

        if tick == 80 { external_load = -50.0; }
        if tick == 150 { external_load = 0.0; }

        let control_effort = pid.update(target_setpoint, motor.speed, dt);
        let current_speed = motor.step(control_effort, external_load, dt);

        time_data.push(time);
        speed_data.push(current_speed);
    }

    // 2. Set up the JPEG drawing canvas (800x600 pixels)
    let root = BitMapBackend::new("pid_results.jpg", (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    // 3. Configure the chart axes parameters
    let mut chart = ChartBuilder::on(&root)
        .caption("PID Motor Speed Simulation Response", ("sans-serif", 30).into_font())
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(0.0f32..10.0f32, 0.0f32..140.0f32)?;

    chart.configure_mesh()
        .x_desc("Time (seconds)")
        .y_desc("Motor Speed (RPM)")
        .draw()?;

    // 4. Draw the target setpoint line (Constant 100)
    chart.draw_series(LineSeries::new(
        vec![(0.0, target_setpoint), (10.0, target_setpoint)],
        &RED,
    ))?
    .label("Setpoint (100 RPM)")
    .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    // 5. Draw the actual calculated motor speed response curve
    let data_points: Vec<(f32, f32)> = time_data.into_iter().zip(speed_data.into_iter()).collect();
    chart.draw_series(LineSeries::new(data_points, &BLUE))?
        .label("Actual Motor Speed")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    // Configure the chart legend location
    chart.configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    root.present()?;
    println!("Success! Simulation graph saved as 'pid_results.jpg'");
    
    Ok(())
}