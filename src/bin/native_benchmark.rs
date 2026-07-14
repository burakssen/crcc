#[cfg(feature = "collide")]
use crcc::benchmark_support::CollideCollisionObject;
#[cfg(feature = "parry")]
use crcc::benchmark_support::ParryCollisionObject;
#[cfg(feature = "rhusics")]
use crcc::benchmark_support::RhusicsCoreCollisionObject;
use crcc::benchmark_support::{
    EngineCollisionObject, SimpleCollisionObject, build_typed, collides, collides_continuous,
    convert_dynamic, distance,
};
use crcc::{
    CollisionCheckerBuilder, CollisionEngine, CollisionObject, CollisionStatus, DynamicObstacle,
    TimeStep,
};
use geo::Rect;
use glamx::DPose2;
use std::hint::black_box;
use std::time::Instant;

const DEFAULT_ITERATIONS: usize = 1_000_000;
const WARMUP_ITERATIONS: usize = 10_000;

#[derive(Clone, Copy)]
enum Layer {
    Native,
    Public,
}

#[derive(Clone, Copy)]
enum Operation {
    Discrete,
    Continuous,
    Distance,
    Dynamic,
}

impl Operation {
    fn name(self) -> &'static str {
        match self {
            Self::Discrete => "discrete",
            Self::Continuous => "continuous",
            Self::Distance => "distance",
            Self::Dynamic => "dynamic",
        }
    }
}

struct Workload {
    name: &'static str,
    operation: Operation,
    left: CollisionObject,
    right: CollisionObject,
    left_start: DPose2,
    left_end: DPose2,
    right_start: DPose2,
    right_end: DPose2,
    trajectory_steps: usize,
    motion_kind: &'static str,
    shape_variation: &'static str,
}

fn main() {
    let mut args = std::env::args().skip(1);
    let backend = args.next().unwrap_or_else(|| usage());
    let layer = match args.next().as_deref() {
        Some("native") => Layer::Native,
        Some("public") => Layer::Public,
        _ => usage(),
    };
    let workload = workload(&args.next().unwrap_or_else(|| usage()));
    let iterations = args
        .next()
        .map(|value| {
            value
                .parse()
                .expect("iterations must be a positive integer")
        })
        .unwrap_or(DEFAULT_ITERATIONS);
    assert!(iterations > 0, "iterations must be positive");
    if args.next().is_some() {
        usage();
    }

    // ponytail: one backend and layer per process keeps native profiles attributable.
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
        _ => panic!("backend is unknown or not enabled: {backend}"),
    }
}

fn run<E: EngineCollisionObject>(
    engine: CollisionEngine,
    backend: &str,
    layer: Layer,
    workload: &Workload,
    iterations: usize,
) {
    let (total_ns, checksum) = match layer {
        Layer::Native => run_native::<E>(workload, iterations),
        Layer::Public => run_public(engine, workload, iterations),
    };
    let layer = match layer {
        Layer::Native => "engine_native",
        Layer::Public => "rust_public_convert_and_query",
    };
    println!(
        "execution_layer,backend,operation,workload,iterations,total_ns,ns_per_query,checksum,trajectory_steps,motion_kind,shape_variation"
    );
    println!(
        "{layer},{backend},{},{},{iterations},{total_ns},{:.3},{checksum},{},{},{}",
        workload.operation.name(),
        workload.name,
        total_ns as f64 / iterations as f64,
        workload.trajectory_steps,
        workload.motion_kind,
        workload.shape_variation,
    );
}

fn run_native<E: EngineCollisionObject>(workload: &Workload, iterations: usize) -> (u128, u64) {
    if matches!(workload.operation, Operation::Dynamic) {
        return run_native_dynamic::<E>(workload, iterations);
    }
    let left = E::from(workload.left.clone());
    let right = E::from(workload.right.clone());
    let execute = || execute_native(&left, &right, workload);
    warm_up(execute);
    let start = Instant::now();
    let checksum = repeat(iterations, execute);
    (start.elapsed().as_nanos(), checksum)
}

fn run_public(engine: CollisionEngine, workload: &Workload, iterations: usize) -> (u128, u64) {
    if matches!(workload.operation, Operation::Dynamic) {
        return run_public_dynamic(engine, workload, iterations);
    }
    let execute = || execute_public(engine, workload);
    warm_up(execute);
    let start = Instant::now();
    let checksum = repeat(iterations, execute);
    (start.elapsed().as_nanos(), checksum)
}

fn execute_native<E: EngineCollisionObject>(left: &E, right: &E, workload: &Workload) -> u64 {
    match workload.operation {
        Operation::Discrete => {
            E::collides_at(left, workload.left_start, right, workload.right_start)
                .expect("native discrete query failed") as u64
        }
        Operation::Continuous => E::collides_continuous(
            left,
            workload.left_start,
            workload.left_end,
            right,
            workload.right_start,
            workload.right_end,
        )
        .expect("native continuous query failed") as u64,
        Operation::Distance => {
            E::distance_at(left, workload.left_start, right, workload.right_start)
                .expect("native distance query failed")
                .to_bits()
        }
        Operation::Dynamic => unreachable!("dynamic workloads use the checker path"),
    }
}

fn execute_public(engine: CollisionEngine, workload: &Workload) -> u64 {
    match workload.operation {
        Operation::Discrete => collides(
            &workload.left,
            workload.left_start,
            &workload.right,
            workload.right_start,
            engine,
        )
        .expect("public discrete query failed") as u64,
        Operation::Continuous => collides_continuous(
            &workload.left,
            workload.left_start,
            workload.left_end,
            &workload.right,
            workload.right_start,
            workload.right_end,
            engine,
        )
        .expect("public continuous query failed") as u64,
        Operation::Distance => distance(
            &workload.left,
            workload.left_start,
            &workload.right,
            workload.right_start,
            engine,
        )
        .expect("public distance query failed")
        .to_bits(),
        Operation::Dynamic => unreachable!("dynamic workloads use the checker path"),
    }
}

fn run_native_dynamic<E: EngineCollisionObject>(
    workload: &Workload,
    iterations: usize,
) -> (u128, u64) {
    let checker = build_typed::<E>(
        CollisionCheckerBuilder::new().with_static_obstacle(workload.right.clone()),
    );
    let query = convert_dynamic::<E>(dynamic_query(workload));
    let execute = || {
        status_checksum(
            checker
                .collides_dynamic(&query)
                .expect("native dynamic query failed"),
        )
    };
    warm_up(execute);
    let start = Instant::now();
    let checksum = repeat(iterations, execute);
    (start.elapsed().as_nanos(), checksum)
}

fn run_public_dynamic(
    engine: CollisionEngine,
    workload: &Workload,
    iterations: usize,
) -> (u128, u64) {
    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(workload.right.clone())
        .build_with_engine(engine)
        .expect("public dynamic checker construction failed");
    let query = dynamic_query(workload);
    let execute = || {
        status_checksum(
            checker
                .collides_dynamic(&query)
                .expect("public dynamic query failed"),
        )
    };
    warm_up(execute);
    let start = Instant::now();
    let checksum = repeat(iterations, execute);
    (start.elapsed().as_nanos(), checksum)
}

fn dynamic_query(workload: &Workload) -> DynamicObstacle {
    let positions = (0..workload.trajectory_steps)
        .map(|step| {
            let t = step as f64 / (workload.trajectory_steps - 1) as f64;
            DPose2::new(((t * 12.0 - 6.0), (t * 2.0 - 1.0)).into(), t * 0.4)
        })
        .collect::<Vec<_>>();
    if workload.shape_variation == "time_variant" {
        let obstacles = (0..workload.trajectory_steps)
            .map(|step| {
                let radius = 0.35 + (step % 4) as f64 * 0.15;
                CollisionObject::circle((0.0, 0.0), radius).unwrap()
            })
            .collect();
        DynamicObstacle::time_variant(obstacles, positions, TimeStep::ZERO)
    } else {
        DynamicObstacle::new(workload.left.clone(), positions, TimeStep::ZERO)
    }
}

fn status_checksum(status: CollisionStatus) -> u64 {
    match status {
        CollisionStatus::NoCollision => 0,
        CollisionStatus::CollidesStatic => 1,
        CollisionStatus::CollidesDynamic(time) => 2_u64.wrapping_add(time.0 as u64),
    }
}

fn warm_up(mut execute: impl FnMut() -> u64) {
    black_box(repeat(WARMUP_ITERATIONS, &mut execute));
}

fn repeat(iterations: usize, mut execute: impl FnMut() -> u64) -> u64 {
    let mut checksum = 0_u64;
    for _ in 0..iterations {
        checksum = checksum.wrapping_add(black_box(execute()));
    }
    checksum
}

fn workload(name: &str) -> Workload {
    let circle = || CollisionObject::circle((0.0, 0.0), 1.0).unwrap();
    let rectangle =
        || CollisionObject::rectangle(Rect::new((-1.0, -0.5), (1.0, 0.5)), 0.2).unwrap();
    let compound = || {
        CollisionObject::merge_all([
            circle(),
            CollisionObject::rectangle(Rect::new((1.5, -0.5), (3.5, 0.5)), 0.2).unwrap(),
            CollisionObject::from(
                SimpleCollisionObject::triangle(geo::Triangle::new(
                    (4.0, -0.5).into(),
                    (5.0, 0.0).into(),
                    (4.0, 0.5).into(),
                ))
                .unwrap(),
            ),
        ])
    };
    let make =
        |name, operation, left, right, left_start, left_end, right_start, right_end| Workload {
            name,
            operation,
            left,
            right,
            left_start,
            left_end,
            right_start,
            right_end,
            trajectory_steps: 0,
            motion_kind: if matches!(operation, Operation::Continuous) {
                "continuous_pose"
            } else {
                "static"
            },
            shape_variation: "fixed",
        };
    match name {
        "circle_clear" => make(
            "circle_clear",
            Operation::Discrete,
            circle(),
            circle(),
            DPose2::IDENTITY,
            DPose2::IDENTITY,
            DPose2::translation(4.0, 0.0),
            DPose2::translation(4.0, 0.0),
        ),
        "circle_hit" => make(
            "circle_hit",
            Operation::Discrete,
            circle(),
            circle(),
            DPose2::IDENTITY,
            DPose2::IDENTITY,
            DPose2::translation(1.0, 0.0),
            DPose2::translation(1.0, 0.0),
        ),
        "rectangle_clear" => make(
            "rectangle_clear",
            Operation::Discrete,
            rectangle(),
            rectangle(),
            DPose2::IDENTITY,
            DPose2::IDENTITY,
            DPose2::translation(4.0, 0.0),
            DPose2::translation(4.0, 0.0),
        ),
        "rectangle_hit" => make(
            "rectangle_hit",
            Operation::Discrete,
            rectangle(),
            rectangle(),
            DPose2::IDENTITY,
            DPose2::IDENTITY,
            DPose2::translation(1.0, 0.0),
            DPose2::translation(1.0, 0.0),
        ),
        "compound_clear" => make(
            "compound_clear",
            Operation::Discrete,
            compound(),
            compound(),
            DPose2::IDENTITY,
            DPose2::IDENTITY,
            DPose2::translation(20.0, 0.0),
            DPose2::translation(20.0, 0.0),
        ),
        "ccd" => make(
            "ccd",
            Operation::Continuous,
            circle(),
            rectangle(),
            DPose2::translation(-4.0, 0.0),
            DPose2::translation(4.0, 0.0),
            DPose2::IDENTITY,
            DPose2::IDENTITY,
        ),
        "tunneling" => make(
            "tunneling",
            Operation::Continuous,
            circle(),
            rectangle(),
            DPose2::translation(-4.0, 0.0),
            DPose2::translation(4.0, 0.0),
            DPose2::IDENTITY,
            DPose2::IDENTITY,
        ),
        "moving_vs_moving" => make(
            "moving_vs_moving",
            Operation::Continuous,
            circle(),
            circle(),
            DPose2::translation(-4.0, 0.0),
            DPose2::translation(4.0, 0.0),
            DPose2::translation(4.0, 0.0),
            DPose2::translation(-4.0, 0.0),
        ),
        "rotation_wrap" => make(
            "rotation_wrap",
            Operation::Continuous,
            rectangle(),
            circle(),
            DPose2::new((0.0, 0.0).into(), std::f64::consts::PI - 0.1),
            DPose2::new((0.0, 0.0).into(), -std::f64::consts::PI + 0.1),
            DPose2::translation(0.0, 4.0),
            DPose2::translation(0.0, 4.0),
        ),
        "endpoint_touch" => make(
            "endpoint_touch",
            Operation::Continuous,
            circle(),
            circle(),
            DPose2::translation(-3.0, 0.0),
            DPose2::IDENTITY,
            DPose2::translation(2.0, 0.0),
            DPose2::translation(2.0, 0.0),
        ),
        "distance" => make(
            "distance",
            Operation::Distance,
            compound(),
            compound(),
            DPose2::IDENTITY,
            DPose2::IDENTITY,
            DPose2::translation(20.0, 0.0),
            DPose2::translation(20.0, 0.0),
        ),
        "dynamic_fixed" | "dynamic_time_variant" => Workload {
            name: if name == "dynamic_fixed" {
                "dynamic_fixed"
            } else {
                "dynamic_time_variant"
            },
            operation: Operation::Dynamic,
            left: circle(),
            right: rectangle(),
            left_start: DPose2::IDENTITY,
            left_end: DPose2::IDENTITY,
            right_start: DPose2::IDENTITY,
            right_end: DPose2::IDENTITY,
            trajectory_steps: 16,
            motion_kind: "translating_rotating",
            shape_variation: if name == "dynamic_fixed" {
                "fixed"
            } else {
                "time_variant"
            },
        },
        _ => usage(),
    }
}

fn usage() -> ! {
    panic!(
        "usage: native_benchmark <parry|rhusics|collide> <native|public> \
         <circle_clear|circle_hit|rectangle_clear|rectangle_hit|compound_clear|ccd|tunneling|moving_vs_moving|rotation_wrap|endpoint_touch|distance|dynamic_fixed|dynamic_time_variant> [iterations]"
    )
}
