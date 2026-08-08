use crcc::{
    CollisionCheckerBuilder, CollisionEngine, CollisionObject, CollisionStatus, CrccError,
    DynamicObstacle, SelectedCollisionChecker, TimeStep,
};
use glamx::DPose2;
use rayon::{ThreadPool, ThreadPoolBuilder};
use std::error::Error;
use std::fmt::{Display, Formatter};
use std::hint::black_box;
use std::process::ExitCode;
use std::time::Instant;

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
enum Operation {
    Static,
    Dynamic,
}

impl Operation {
    fn parse(value: &str) -> Option<Self> {
        match value {
            "static" => Some(Self::Static),
            "dynamic" => Some(Self::Dynamic),
            _ => None,
        }
    }

    const fn name(self) -> &'static str {
        match self {
            Self::Static => "static",
            Self::Dynamic => "dynamic",
        }
    }
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
    let mut arguments = std::env::args().skip(1);

    let engine_name = arguments.next().ok_or_else(usage)?;
    let engine = parse_engine(&engine_name)?;

    let operation = arguments
        .next()
        .as_deref()
        .and_then(Operation::parse)
        .ok_or_else(usage)?;

    let batch_size = parse_positive(&mut arguments, "batch size")?;
    let threads = parse_positive(&mut arguments, "thread count")?;
    let iterations = parse_positive(&mut arguments, "iterations")?;

    if arguments.next().is_some() {
        return Err(usage());
    }

    let static_obstacle = CollisionObject::circle((0.0, 0.0), 0.75)?;

    let checker = CollisionCheckerBuilder::new()
        .with_static_obstacle(static_obstacle)
        .build_with_engine(engine)?;

    let pool = ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .map_err(|error| invalid_input(format!("thread-pool construction failed: {error}")))?;

    let (scalar_ns, batch_ns, checksum) = match operation {
        Operation::Static => benchmark_static(&checker, &pool, batch_size, iterations)?,
        Operation::Dynamic => benchmark_dynamic(&checker, &pool, batch_size, iterations)?,
    };

    print_results(
        operation, batch_size, threads, iterations, scalar_ns, batch_ns, checksum,
    )
}

fn parse_engine(value: &str) -> BenchmarkResult<CollisionEngine> {
    match value {
        "parry" => Ok(CollisionEngine::Parry),
        "rhusics" => Ok(CollisionEngine::Rhusics),
        "collide" => Ok(CollisionEngine::Collide),
        _ => Err(invalid_input(format!("unknown collision engine: {value}"))),
    }
}

fn benchmark_static(
    checker: &SelectedCollisionChecker,
    pool: &ThreadPool,
    batch_size: usize,
    iterations: usize,
) -> BenchmarkResult<(u128, u128, u64)> {
    let query_shape = CollisionObject::circle((0.0, 0.0), 0.5)?;

    let queries = (0..batch_size)
        .map(|index| {
            let x = if index.is_multiple_of(2) { 0.0 } else { 4.0 };

            (query_shape.clone(), DPose2::translation(x, 0.0))
        })
        .collect::<Vec<_>>();

    let sequential = || {
        queries
            .iter()
            .map(|(shape, pose)| {
                checker
                    .collides_static_range(shape, *pose, ..)
                    .map_err(BenchmarkError::from)
            })
            .collect::<BenchmarkResult<Vec<_>>>()
    };

    let parallel = || pool.install(|| checker.collides_static_batch(&queries, ..));

    measure(sequential, parallel, iterations)
}

fn benchmark_dynamic(
    checker: &SelectedCollisionChecker,
    pool: &ThreadPool,
    batch_size: usize,
    iterations: usize,
) -> BenchmarkResult<(u128, u128, u64)> {
    let query_shape = CollisionObject::circle((0.0, 0.0), 0.5)?;

    let queries = (0..batch_size)
        .map(|index| {
            let x = if index.is_multiple_of(2) { 4.0 } else { 10.0 };

            DynamicObstacle::new(
                query_shape.clone(),
                vec![DPose2::translation(x, 0.0), DPose2::translation(0.0, 0.0)],
                TimeStep::ZERO,
            )
        })
        .collect::<Result<Vec<_>, _>>()?;

    let sequential = || {
        queries
            .iter()
            .map(|query| {
                checker
                    .collides_dynamic(query)
                    .map_err(BenchmarkError::from)
            })
            .collect::<BenchmarkResult<Vec<_>>>()
    };

    let parallel = || pool.install(|| checker.collides_dynamic_batch(&queries, ..));

    measure(sequential, parallel, iterations)
}

fn measure(
    mut sequential: impl FnMut() -> BenchmarkResult<Vec<CollisionStatus>>,
    mut parallel: impl FnMut() -> Vec<Result<CollisionStatus, CrccError>>,
    iterations: usize,
) -> BenchmarkResult<(u128, u128, u64)> {
    let expected = sequential()?;

    let actual = parallel().into_iter().collect::<Result<Vec<_>, _>>()?;

    if actual != expected {
        return Err(invalid_input(
            "parallel results differ from sequential results",
        ));
    }

    let mut checksum = 0_u64;

    let start = Instant::now();

    for _ in 0..iterations {
        let values = sequential()?;
        checksum = checksum.wrapping_add(status_checksum(black_box(values)));
    }

    let scalar_ns = start.elapsed().as_nanos();
    let start = Instant::now();

    for _ in 0..iterations {
        let values = parallel().into_iter().collect::<Result<Vec<_>, _>>()?;

        checksum = checksum.wrapping_add(status_checksum(black_box(values)));
    }

    let batch_ns = start.elapsed().as_nanos();

    Ok((scalar_ns, batch_ns, checksum))
}

fn status_checksum(values: Vec<CollisionStatus>) -> u64 {
    values.into_iter().fold(0_u64, |checksum, status| {
        checksum.wrapping_add(status_value(status))
    })
}

fn status_value(status: CollisionStatus) -> u64 {
    match status {
        CollisionStatus::NoCollision => 0,
        CollisionStatus::CollidesStatic => 1,
        CollisionStatus::CollidesDynamic(time) => {
            let encoded_time = u64::from(u32::from_ne_bytes(time.0.to_ne_bytes()));

            2_u64.wrapping_add(encoded_time)
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn print_results(
    operation: Operation,
    batch_size: usize,
    threads: usize,
    iterations: usize,
    scalar_ns: u128,
    batch_ns: u128,
    checksum: u64,
) -> BenchmarkResult<()> {
    let query_count = batch_size
        .checked_mul(iterations)
        .ok_or_else(|| invalid_input("batch size multiplied by iterations overflowed"))?;

    let scalar_ns_per_query = format_ns_per_query(scalar_ns, query_count)?;

    let batch_ns_per_query = format_ns_per_query(batch_ns, query_count)?;

    println!(
        "operation,api_mode,batch_size,threads,iterations,\
         total_ns,ns_per_query,checksum"
    );

    println!(
        "{},scalar,{batch_size},1,{iterations},\
         {scalar_ns},{scalar_ns_per_query},{checksum}",
        operation.name(),
    );

    println!(
        "{},batch_reusable,{batch_size},{threads},{iterations},\
         {batch_ns},{batch_ns_per_query},{checksum}",
        operation.name(),
    );

    Ok(())
}

fn format_ns_per_query(total_ns: u128, query_count: usize) -> BenchmarkResult<String> {
    let divisor = u128::try_from(query_count)
        .map_err(|error| invalid_input(format!("query count does not fit into u128: {error}")))?;

    let whole = total_ns
        .checked_div(divisor)
        .ok_or_else(|| invalid_input("query count must be positive"))?;

    let remainder = total_ns
        .checked_rem(divisor)
        .ok_or_else(|| invalid_input("query count must be positive"))?;

    let scaled_remainder = remainder
        .checked_mul(1_000)
        .ok_or_else(|| invalid_input("nanoseconds-per-query calculation overflowed"))?;

    let fractional = scaled_remainder
        .checked_div(divisor)
        .ok_or_else(|| invalid_input("query count must be positive"))?;

    Ok(format!("{whole}.{fractional:03}"))
}

fn parse_positive(
    arguments: &mut impl Iterator<Item = String>,
    name: &str,
) -> BenchmarkResult<usize> {
    let value = arguments.next().ok_or_else(usage)?;

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
        "usage: parallel_benchmark \
         <parry|rhusics|collide> \
         <static|dynamic> \
         <batch-size> <threads> <iterations>",
    )
}
