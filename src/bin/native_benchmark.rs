#[cfg(feature = "collide")]
use crcc::benchmark_support::CollideCollisionObject;
#[cfg(feature = "parry")]
use crcc::benchmark_support::ParryCollisionObject;
#[cfg(feature = "rhusics")]
use crcc::benchmark_support::RhusicsCoreCollisionObject;
use crcc::benchmark_support::{
    EngineCollisionObject, build_typed, collides, collides_continuous, convert_dynamic, distance,
};
use crcc::{
    CollisionCheckerBuilder, CollisionEngine, CollisionObject, CollisionStatus, CrccError,
    DynamicObstacle, TimeStep,
};
use geo::Rect;
use glamx::DPose2;
use std::error::Error;
use std::fmt::{Display, Formatter};
use std::hint::black_box;
use std::ops::{Add, Div, Neg, Sub};
use std::process::ExitCode;
use std::time::Instant;

const DEFAULT_ITERATIONS: usize = 1_000_000;
const WARMUP_ITERATIONS: usize = 10_000;

#[derive(Debug)]
enum BenchmarkError {
    Collision(CrccError),
    InvalidInput(String),
}

impl Display for BenchmarkError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Collision(error) => Display::fmt(error, formatter),
            Self::InvalidInput(message) => formatter.write_str(message),
        }
    }
}

impl Error for BenchmarkError {}

impl From<CrccError> for BenchmarkError {
    fn from(error: CrccError) -> Self {
        Self::Collision(error)
    }
}

type BenchmarkResult<T> = Result<T, BenchmarkError>;

#[derive(Clone, Copy)]
enum Layer {
    Native,
    Public,
}

impl Layer {
    fn parse(value: &str) -> Option<Self> {
        match value {
            "native" => Some(Self::Native),
            "public" => Some(Self::Public),
            _ => None,
        }
    }

    const fn name(self) -> &'static str {
        match self {
            Self::Native => "engine_native",
            Self::Public => "rust_public_convert_and_query",
        }
    }
}

#[derive(Clone, Copy)]
enum Operation {
    Discrete,
    Continuous,
    Distance,
    Dynamic,
}

impl Operation {
    const fn name(self) -> &'static str {
        match self {
            Self::Discrete => "discrete",
            Self::Continuous => "continuous",
            Self::Distance => "distance",
            Self::Dynamic => "dynamic",
        }
    }
}

#[derive(Clone, Copy)]
struct Motion {
    left_start: DPose2,
    left_end: DPose2,
    right_start: DPose2,
    right_end: DPose2,
}

struct Workload {
    name: &'static str,
    operation: Operation,
    left: CollisionObject,
    right: CollisionObject,
    motion: Motion,
    trajectory_steps: usize,
    motion_kind: &'static str,
    shape_variation: &'static str,
}

fn main() -> ExitCode {
    match run_cli() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

fn run_cli() -> BenchmarkResult<()> {
    let mut args = std::env::args().skip(1);
    let backend = args.next().ok_or_else(usage)?;
    let layer = args
        .next()
        .as_deref()
        .and_then(Layer::parse)
        .ok_or_else(usage)?;
    let workload_name = args.next().ok_or_else(usage)?;
    let workload = workload(&workload_name)?;
    let iterations = args.next().map_or(Ok(DEFAULT_ITERATIONS), |value| {
        parse_positive_usize(&value, "iterations")
    })?;

    if args.next().is_some() {
        return Err(usage());
    }

    match backend.as_str() {
        #[cfg(feature = "parry")]
        "parry" => run::<ParryCollisionObject>(
            CollisionEngine::Parry,
            "parry",
            layer,
            &workload,
            iterations,
        ),
        #[cfg(feature = "rhusics")]
        "rhusics" => run::<RhusicsCoreCollisionObject>(
            CollisionEngine::Rhusics,
            "rhusics",
            layer,
            &workload,
            iterations,
        ),
        #[cfg(feature = "collide")]
        "collide" => run::<CollideCollisionObject>(
            CollisionEngine::Collide,
            "collide",
            layer,
            &workload,
            iterations,
        ),
        _ => Err(invalid_input(format!(
            "backend is unknown or not enabled: {backend}"
        ))),
    }
}

fn run<E: EngineCollisionObject>(
    engine: CollisionEngine,
    backend: &str,
    layer: Layer,
    workload: &Workload,
    iterations: usize,
) -> BenchmarkResult<()> {
    let (total_ns, checksum) = match layer {
        Layer::Native => run_native::<E>(workload, iterations)?,
        Layer::Public => run_public(engine, workload, iterations)?,
    };
    let ns_per_query = format_ns_per_query(total_ns, iterations)?;

    println!(
        "execution_layer,backend,operation,workload,iterations,total_ns,ns_per_query,checksum,trajectory_steps,motion_kind,shape_variation"
    );
    println!(
        "{},{backend},{},{},{iterations},{total_ns},{ns_per_query},{checksum},{},{},{}",
        layer.name(),
        workload.operation.name(),
        workload.name,
        workload.trajectory_steps,
        workload.motion_kind,
        workload.shape_variation,
    );

    Ok(())
}

fn format_ns_per_query(total_ns: u128, iterations: usize) -> BenchmarkResult<String> {
    let divisor = u128::try_from(iterations).map_err(|error| {
        invalid_input(format!("iteration count does not fit into u128: {error}"))
    })?;

    let whole = total_ns
        .checked_div(divisor)
        .ok_or_else(|| invalid_input("iterations must be positive"))?;

    let remainder = total_ns
        .checked_rem(divisor)
        .ok_or_else(|| invalid_input("iterations must be positive"))?;

    let scaled_remainder = remainder
        .checked_mul(1_000)
        .ok_or_else(|| invalid_input("nanoseconds-per-query calculation overflowed"))?;

    let fractional = scaled_remainder
        .checked_div(divisor)
        .ok_or_else(|| invalid_input("iterations must be positive"))?;

    Ok(format!("{whole}.{fractional:03}"))
}

fn run_native<E: EngineCollisionObject>(
    workload: &Workload,
    iterations: usize,
) -> BenchmarkResult<(u128, u64)> {
    if matches!(workload.operation, Operation::Dynamic) {
        return run_native_dynamic::<E>(workload, iterations);
    }

    let left: E = workload.left.clone().into();
    let right: E = workload.right.clone().into();
    let mut execute = || execute_native(&left, &right, workload);

    warm_up(&mut execute)?;
    let start = Instant::now();
    let checksum = repeat(iterations, &mut execute)?;
    Ok((start.elapsed().as_nanos(), checksum))
}

fn run_public(
    engine: CollisionEngine,
    workload: &Workload,
    iterations: usize,
) -> BenchmarkResult<(u128, u64)> {
    if matches!(workload.operation, Operation::Dynamic) {
        return run_public_dynamic(engine, workload, iterations);
    }

    let mut execute = || execute_public(engine, workload);
    warm_up(&mut execute)?;
    let start = Instant::now();
    let checksum = repeat(iterations, &mut execute)?;
    Ok((start.elapsed().as_nanos(), checksum))
}

fn execute_native<E: EngineCollisionObject>(
    left: &E,
    right: &E,
    workload: &Workload,
) -> BenchmarkResult<u64> {
    let motion = workload.motion;

    match workload.operation {
        Operation::Discrete => Ok(u64::from(E::collides_at(
            left,
            motion.left_start,
            right,
            motion.right_start,
        )?)),
        Operation::Continuous => Ok(u64::from(E::collides_continuous(
            left,
            motion.left_start,
            motion.left_end,
            right,
            motion.right_start,
            motion.right_end,
        )?)),
        Operation::Distance => {
            Ok(E::distance_at(left, motion.left_start, right, motion.right_start)?.to_bits())
        }
        Operation::Dynamic => Err(invalid_input(
            "dynamic workloads must use the checker benchmark path",
        )),
    }
}

fn execute_public(engine: CollisionEngine, workload: &Workload) -> BenchmarkResult<u64> {
    let motion = workload.motion;

    match workload.operation {
        Operation::Discrete => Ok(u64::from(collides(
            &workload.left,
            motion.left_start,
            &workload.right,
            motion.right_start,
            engine,
        )?)),
        Operation::Continuous => Ok(u64::from(collides_continuous(
            &workload.left,
            motion.left_start,
            motion.left_end,
            &workload.right,
            motion.right_start,
            motion.right_end,
            engine,
        )?)),
        Operation::Distance => Ok(distance(
            &workload.left,
            motion.left_start,
            &workload.right,
            motion.right_start,
            engine,
        )?
        .to_bits()),
        Operation::Dynamic => Err(invalid_input(
            "dynamic workloads must use the checker benchmark path",
        )),
    }
}

fn run_native_dynamic<E: EngineCollisionObject>(
    workload: &Workload,
    iterations: usize,
) -> BenchmarkResult<(u128, u64)> {
    let checker = build_typed::<E>(
        CollisionCheckerBuilder::new().with_static_obstacle(workload.right.clone()),
    );
    let query = convert_dynamic::<E>(dynamic_query(workload)?);
    let mut execute = || {
        checker
            .collides_dynamic(&query)
            .map(status_checksum)
            .map_err(BenchmarkError::from)
    };

    warm_up(&mut execute)?;
    let start = Instant::now();
    let checksum = repeat(iterations, &mut execute)?;
    Ok((start.elapsed().as_nanos(), checksum))
}

fn run_public_dynamic(
    engine: CollisionEngine,
    workload: &Workload,
    iterations: usize,
) -> BenchmarkResult<(u128, u64)> {
    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(workload.right.clone())
        .build_with_engine(engine)?;
    let query = dynamic_query(workload)?;
    let mut execute = || {
        checker
            .collides_dynamic(&query)
            .map(status_checksum)
            .map_err(BenchmarkError::from)
    };

    warm_up(&mut execute)?;
    let start = Instant::now();
    let checksum = repeat(iterations, &mut execute)?;
    Ok((start.elapsed().as_nanos(), checksum))
}

fn dynamic_query(workload: &Workload) -> BenchmarkResult<DynamicObstacle> {
    if workload.trajectory_steps < 2 {
        return Err(invalid_input(
            "dynamic workloads require at least two trajectory steps",
        ));
    }

    let last_step = workload
        .trajectory_steps
        .checked_sub(1)
        .ok_or_else(|| invalid_input("trajectory step count underflowed"))?;
    let denominator = f64::from(
        u32::try_from(last_step).map_err(|_| invalid_input("trajectory step count exceeds u32"))?,
    );

    let positions = (0..workload.trajectory_steps)
        .map(|step| {
            let step = u32::try_from(step)
                .map_err(|_| invalid_input("trajectory step index exceeds u32"))?;
            let interpolation = f64::from(step).div(denominator);
            let x = interpolation.mul_add(12.0, -6.0);
            let y = interpolation.mul_add(2.0, -1.0);
            let rotation = interpolation.mul_add(0.4, 0.0);
            Ok(DPose2::new((x, y).into(), rotation))
        })
        .collect::<BenchmarkResult<Vec<_>>>()?;

    if workload.shape_variation == "time_variant" {
        let obstacles = (0..workload.trajectory_steps)
            .map(|step| {
                let phase = u32::try_from(step.rem_euclid(4))
                    .map_err(|_| invalid_input("shape-variation phase exceeds u32"))?;
                let radius = f64::from(phase).mul_add(0.15, 0.35);
                CollisionObject::circle((0.0, 0.0), radius).map_err(BenchmarkError::from)
            })
            .collect::<BenchmarkResult<Vec<_>>>()?;

        DynamicObstacle::time_variant(obstacles, positions, TimeStep::ZERO)
            .map_err(BenchmarkError::from)
    } else {
        DynamicObstacle::new(workload.left.clone(), positions, TimeStep::ZERO)
            .map_err(BenchmarkError::from)
    }
}

fn status_checksum(status: CollisionStatus) -> u64 {
    match status {
        CollisionStatus::NoCollision => 0,
        CollisionStatus::CollidesStatic => 1,
        CollisionStatus::CollidesDynamic(time) => {
            let encoded_time = u64::from(u32::from_ne_bytes(time.0.to_ne_bytes()));
            2_u64.wrapping_add(encoded_time)
        }
    }
}

fn warm_up(execute: impl FnMut() -> BenchmarkResult<u64>) -> BenchmarkResult<()> {
    black_box(repeat(WARMUP_ITERATIONS, execute)?);
    Ok(())
}

fn repeat(
    iterations: usize,
    mut execute: impl FnMut() -> BenchmarkResult<u64>,
) -> BenchmarkResult<u64> {
    let mut checksum = 0_u64;

    for _ in 0..iterations {
        checksum = checksum.wrapping_add(black_box(execute()?));
    }

    Ok(checksum)
}

fn workload(name: &str) -> BenchmarkResult<Workload> {
    match name {
        "circle_clear" | "circle_hit" | "rectangle_clear" | "rectangle_hit" | "compound_clear" => {
            discrete_workload(name)
        }
        "ccd" | "tunneling" | "moving_vs_moving" | "rotation_wrap" | "endpoint_touch" => {
            continuous_workload(name)
        }
        "distance" => distance_workload(),
        "dynamic_fixed" | "dynamic_time_variant" => dynamic_workload(name),
        _ => Err(usage()),
    }
}

fn discrete_workload(name: &str) -> BenchmarkResult<Workload> {
    match name {
        "circle_clear" => Ok(make_workload(
            "circle_clear",
            Operation::Discrete,
            circle()?,
            circle()?,
            stationary_motion(DPose2::translation(4.0, 0.0)),
        )),
        "circle_hit" => Ok(make_workload(
            "circle_hit",
            Operation::Discrete,
            circle()?,
            circle()?,
            stationary_motion(DPose2::translation(1.0, 0.0)),
        )),
        "rectangle_clear" => Ok(make_workload(
            "rectangle_clear",
            Operation::Discrete,
            rectangle()?,
            rectangle()?,
            stationary_motion(DPose2::translation(4.0, 0.0)),
        )),
        "rectangle_hit" => Ok(make_workload(
            "rectangle_hit",
            Operation::Discrete,
            rectangle()?,
            rectangle()?,
            stationary_motion(DPose2::translation(1.0, 0.0)),
        )),
        "compound_clear" => Ok(make_workload(
            "compound_clear",
            Operation::Discrete,
            compound()?,
            compound()?,
            stationary_motion(DPose2::translation(20.0, 0.0)),
        )),
        _ => Err(usage()),
    }
}

fn continuous_workload(name: &str) -> BenchmarkResult<Workload> {
    match name {
        "ccd" => Ok(make_workload(
            "ccd",
            Operation::Continuous,
            circle()?,
            rectangle()?,
            crossing_motion(),
        )),
        "tunneling" => Ok(make_workload(
            "tunneling",
            Operation::Continuous,
            circle()?,
            rectangle()?,
            crossing_motion(),
        )),
        "moving_vs_moving" => Ok(make_workload(
            "moving_vs_moving",
            Operation::Continuous,
            circle()?,
            circle()?,
            Motion {
                left_start: DPose2::translation(-4.0, 0.0),
                left_end: DPose2::translation(4.0, 0.0),
                right_start: DPose2::translation(4.0, 0.0),
                right_end: DPose2::translation(-4.0, 0.0),
            },
        )),
        "rotation_wrap" => Ok(make_workload(
            "rotation_wrap",
            Operation::Continuous,
            rectangle()?,
            circle()?,
            Motion {
                left_start: DPose2::new((0.0, 0.0).into(), std::f64::consts::PI.sub(0.1)),
                left_end: DPose2::new((0.0, 0.0).into(), std::f64::consts::PI.neg().add(0.1)),
                right_start: DPose2::translation(0.0, 4.0),
                right_end: DPose2::translation(0.0, 4.0),
            },
        )),
        "endpoint_touch" => Ok(make_workload(
            "endpoint_touch",
            Operation::Continuous,
            circle()?,
            circle()?,
            Motion {
                left_start: DPose2::translation(-3.0, 0.0),
                left_end: DPose2::IDENTITY,
                right_start: DPose2::translation(2.0, 0.0),
                right_end: DPose2::translation(2.0, 0.0),
            },
        )),
        _ => Err(usage()),
    }
}

fn distance_workload() -> BenchmarkResult<Workload> {
    Ok(make_workload(
        "distance",
        Operation::Distance,
        compound()?,
        compound()?,
        stationary_motion(DPose2::translation(20.0, 0.0)),
    ))
}

fn dynamic_workload(name: &str) -> BenchmarkResult<Workload> {
    let (name, shape_variation) = match name {
        "dynamic_fixed" => ("dynamic_fixed", "fixed"),
        "dynamic_time_variant" => ("dynamic_time_variant", "time_variant"),
        _ => return Err(usage()),
    };

    Ok(Workload {
        name,
        operation: Operation::Dynamic,
        left: circle()?,
        right: rectangle()?,
        motion: stationary_motion(DPose2::IDENTITY),
        trajectory_steps: 16,
        motion_kind: "translating_rotating",
        shape_variation,
    })
}

const fn make_workload(
    name: &'static str,
    operation: Operation,
    left: CollisionObject,
    right: CollisionObject,
    motion: Motion,
) -> Workload {
    Workload {
        name,
        operation,
        left,
        right,
        motion,
        trajectory_steps: 0,
        motion_kind: if matches!(operation, Operation::Continuous) {
            "continuous_pose"
        } else {
            "static"
        },
        shape_variation: "fixed",
    }
}

const fn stationary_motion(right_position: DPose2) -> Motion {
    Motion {
        left_start: DPose2::IDENTITY,
        left_end: DPose2::IDENTITY,
        right_start: right_position,
        right_end: right_position,
    }
}

fn crossing_motion() -> Motion {
    Motion {
        left_start: DPose2::translation(-4.0, 0.0),
        left_end: DPose2::translation(4.0, 0.0),
        right_start: DPose2::IDENTITY,
        right_end: DPose2::IDENTITY,
    }
}

fn circle() -> BenchmarkResult<CollisionObject> {
    CollisionObject::circle((0.0, 0.0), 1.0).map_err(BenchmarkError::from)
}

fn rectangle() -> BenchmarkResult<CollisionObject> {
    CollisionObject::rectangle(Rect::new((-1.0, -0.5), (1.0, 0.5)), 0.2)
        .map_err(BenchmarkError::from)
}

fn compound() -> BenchmarkResult<CollisionObject> {
    Ok(CollisionObject::merge_all([
        circle()?,
        CollisionObject::rectangle(Rect::new((1.5, -0.5), (3.5, 0.5)), 0.2)?,
        CollisionObject::triangle(geo::Triangle::new(
            (4.0, -0.5).into(),
            (5.0, 0.0).into(),
            (4.0, 0.5).into(),
        ))?,
    ]))
}

fn parse_positive_usize(value: &str, name: &str) -> BenchmarkResult<usize> {
    let parsed = value
        .parse::<usize>()
        .map_err(|error| invalid_input(format!("{name} must be a positive integer: {error}")))?;

    if parsed == 0 {
        return Err(invalid_input(format!("{name} must be a positive integer")));
    }

    Ok(parsed)
}

fn invalid_input(message: impl Into<String>) -> BenchmarkError {
    BenchmarkError::InvalidInput(message.into())
}

fn usage() -> BenchmarkError {
    invalid_input(
        "usage: native_benchmark <parry|rhusics|collide> <native|public> \
         <circle_clear|circle_hit|rectangle_clear|rectangle_hit|compound_clear|ccd|tunneling|moving_vs_moving|rotation_wrap|endpoint_touch|distance|dynamic_fixed|dynamic_time_variant> [iterations]",
    )
}
