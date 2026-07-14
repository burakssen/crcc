use crcc::{
    CollisionCheckerBuilder, CollisionEngine, CollisionObject, CollisionStatus, CrccError,
    DynamicObstacle, TimeStep,
};
use glamx::DPose2;
use rayon::ThreadPoolBuilder;
use std::hint::black_box;
use std::time::Instant;

fn main() {
    let mut args = std::env::args().skip(1);
    let engine = match args.next().as_deref() {
        Some("parry") => CollisionEngine::Parry,
        Some("rhusics") => CollisionEngine::Rhusics,
        Some("collide") => CollisionEngine::Collide,
        _ => usage(),
    };
    let operation = args.next().unwrap_or_else(|| usage());
    let batch_size = parse(&mut args, "batch size");
    let threads = parse(&mut args, "thread count");
    let iterations = parse(&mut args, "iterations");
    if args.next().is_some() || batch_size == 0 || threads == 0 || iterations == 0 {
        usage();
    }

    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(CollisionObject::circle((0.0, 0.0), 0.75).unwrap())
        .build_with_engine(engine)
        .expect("checker construction failed");
    let pool = ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .expect("thread-pool construction failed");

    let (scalar_ns, batch_ns, checksum) = match operation.as_str() {
        "static" => {
            let queries = (0..batch_size)
                .map(|index| {
                    (
                        CollisionObject::circle((0.0, 0.0), 0.5).unwrap(),
                        DPose2::translation(if index % 2 == 0 { 0.0 } else { 4.0 }, 0.0),
                    )
                })
                .collect::<Vec<_>>();
            let sequential = || {
                queries
                    .iter()
                    .map(|(shape, pose)| checker.collides_static_range(shape, *pose, ..).unwrap())
                    .collect::<Vec<_>>()
            };
            let parallel = || pool.install(|| checker.collides_static_batch(&queries, ..));
            measure(sequential, parallel, iterations)
        }
        "dynamic" => {
            let queries = (0..batch_size)
                .map(|index| {
                    let x = if index % 2 == 0 { 4.0 } else { 10.0 };
                    DynamicObstacle::new(
                        CollisionObject::circle((0.0, 0.0), 0.5).unwrap(),
                        vec![DPose2::translation(x, 0.0), DPose2::translation(0.0, 0.0)],
                        TimeStep::ZERO,
                    )
                })
                .collect::<Vec<_>>();
            let sequential = || {
                queries
                    .iter()
                    .map(|query| checker.collides_dynamic(query).unwrap())
                    .collect::<Vec<_>>()
            };
            let parallel = || pool.install(|| checker.collides_dynamic_batch(&queries, ..));
            measure(sequential, parallel, iterations)
        }
        _ => usage(),
    };

    println!("operation,api_mode,batch_size,threads,iterations,total_ns,ns_per_query,checksum");
    println!(
        "{operation},scalar,{batch_size},1,{iterations},{scalar_ns},{:.3},{checksum}",
        scalar_ns as f64 / (batch_size * iterations) as f64
    );
    println!(
        "{operation},batch_reusable,{batch_size},{threads},{iterations},{batch_ns},{:.3},{checksum}",
        batch_ns as f64 / (batch_size * iterations) as f64
    );
}

fn measure(
    mut sequential: impl FnMut() -> Vec<CollisionStatus>,
    mut parallel: impl FnMut() -> Vec<Result<CollisionStatus, CrccError>>,
    iterations: usize,
) -> (u128, u128, u64) {
    let expected = sequential();
    let actual = parallel()
        .into_iter()
        .collect::<Result<Vec<_>, _>>()
        .expect("parallel query failed");
    assert_eq!(
        actual, expected,
        "parallel results differ from sequential results"
    );

    let start = Instant::now();
    let mut checksum = 0_u64;
    for _ in 0..iterations {
        checksum = checksum.wrapping_add(status_checksum(black_box(sequential())));
    }
    let scalar_ns = start.elapsed().as_nanos();

    let start = Instant::now();
    for _ in 0..iterations {
        let values = parallel()
            .into_iter()
            .collect::<Result<Vec<_>, _>>()
            .expect("parallel query failed");
        checksum = checksum.wrapping_add(status_checksum(black_box(values)));
    }
    (scalar_ns, start.elapsed().as_nanos(), checksum)
}

fn status_checksum(values: Vec<CollisionStatus>) -> u64 {
    values
        .into_iter()
        .map(|status| match status {
            CollisionStatus::NoCollision => 0,
            CollisionStatus::CollidesStatic => 1,
            CollisionStatus::CollidesDynamic(time) => 2_u64.wrapping_add(time.0 as u64),
        })
        .sum()
}

fn parse(args: &mut impl Iterator<Item = String>, name: &str) -> usize {
    args.next()
        .unwrap_or_else(|| usage())
        .parse()
        .unwrap_or_else(|_| panic!("{name} must be a positive integer"))
}

fn usage() -> ! {
    panic!(
        "usage: parallel_benchmark <parry|rhusics|collide> <static|dynamic> <batch-size> <threads> <iterations>"
    )
}
