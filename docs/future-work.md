## Future Work: Spatial Hashing and Grid-Based Broad Phase

The current swept-AABB optimization efficiently rejects collision pairs whose motion bounds do not overlap. However, the collision checker must still consider every possible pair of dynamic objects before applying this rejection test.

For a scene containing $n$ objects, a naive all-pairs broad phase requires

$$
\frac{n(n-1)}{2}
$$

pair checks, resulting in quadratic complexity:

$$
O(n^2).
$$

This approach is sufficient for small scenes, but it can become a significant bottleneck when simulations contain hundreds or thousands of dynamic objects.

### Proposed Optimization

A future optimization is to introduce a spatial acceleration structure for the broad phase. Possible approaches include:

* **Uniform spatial hashing**
* **Grid-based spatial partitioning**
* **Sweep and Prune**
* **Dynamic AABB trees**

The initial implementation would likely use a spatial hash grid because it is relatively simple, supports dynamic objects efficiently, and integrates naturally with the existing swept-AABB representation.

Each object's swept AABB would be inserted into every grid cell that it overlaps during the current simulation step.

```text
Object motion
     │
     ▼
Compute swept AABB
     │
     ▼
Determine overlapping grid cells
     │
     ▼
Insert object into spatial hash buckets
     │
     ▼
Generate candidate pairs within each bucket
     │
     ▼
Run narrow-phase CCD only for candidate pairs
```

Objects would only be considered as potential collision pairs when their swept AABBs occupy at least one common grid cell.

This avoids testing objects that are located in unrelated regions of the simulation space.

### Candidate-Pair Generation

A simplified version of the process would be:

```rust
for object in objects {
    let swept_bounds = swept_aabb(
        &object.shape,
        object.start_pose,
        object.end_pose,
    );

    for cell in grid.cells_overlapping(&swept_bounds) {
        grid.insert(cell, object.id);
    }
}

for bucket in grid.buckets() {
    for pair in unique_pairs(bucket.objects()) {
        candidate_pairs.insert(pair);
    }
}
```

Because an object may overlap multiple cells, the same pair may be generated more than once. Candidate pairs must therefore be deduplicated before the continuous collision query is executed.

A canonical pair representation can be used:

```rust
let pair = if object_a < object_b {
    (object_a, object_b)
} else {
    (object_b, object_a)
};
```

The pair can then be stored in a hash set.

### Choosing the Grid Cell Size

The performance of a spatial grid depends strongly on the selected cell size.

If cells are too small:

* Large objects overlap many cells.
* The number of grid insertions increases.
* Duplicate candidate pairs become more common.

If cells are too large:

* Too many unrelated objects occupy the same cell.
* Candidate buckets become crowded.
* The broad phase approaches the original all-pairs behavior.

A reasonable initial strategy is to select a cell size based on the typical object diameter:

$$
h \approx 2r_{\mathrm{typical}},
$$

where $h$ is the grid-cell width and $r_{\mathrm{typical}}$ is a representative object radius.

More advanced implementations could adapt the grid resolution dynamically or use a hierarchical spatial hash for scenes containing objects with substantially different sizes.

### Alternative: Sweep and Prune

Sweep and Prune is another suitable broad-phase method.

The algorithm projects each swept AABB onto one or more coordinate axes and sorts the interval endpoints. Candidate pairs are generated only when their projected intervals overlap.

For the $x$-axis, each swept AABB produces an interval

$$
[x_{\min}, x_{\max}].
$$

Objects whose intervals do not overlap cannot collide and can be discarded immediately.

Sweep and Prune is particularly effective when object motion is coherent between consecutive simulation steps because the sorted endpoint list changes only slightly and can be updated incrementally.

However, spatial hashing may be easier to parallelize and may perform better for objects distributed across a large, sparse environment.

### Expected Impact

For well-distributed objects, spatial partitioning can reduce candidate generation from quadratic behavior toward approximately linear expected complexity:

$$
O(n^2) \longrightarrow O(n + k),
$$

where $k$ is the number of candidate pairs that share a spatial region.

The exact performance depends on:

* Object density
* Object-size distribution
* Grid-cell size
* Motion distance during each step
* The number of cells overlapped by each swept AABB

In sparse scenes, most objects occupy separate cells, causing the number of generated candidate pairs to remain small.

In highly dense scenes where every object occupies the same region, the worst-case complexity remains

$$
O(n^2),
$$

because every object may genuinely be a potential collision candidate.

### Implementation Considerations

The implementation should account for the following issues:

1. **Swept bounds rather than static bounds**

   Objects must be inserted using their full swept AABBs. Using only their initial or final AABBs could miss collisions occurring between simulation steps.

2. **Duplicate candidate pairs**

   Objects that overlap multiple cells can produce the same candidate pair repeatedly. Candidate pairs must be deduplicated.

3. **Large objects**

   Large swept AABBs may overlap many cells and reduce the efficiency of a uniform grid. Such objects may require a separate list or a hierarchical grid.

4. **Fast grid reset**

   The spatial structure must be rebuilt or updated for each simulation step. Bucket storage should therefore be reusable to avoid unnecessary allocations.

5. **Parallel construction**

   Grid insertion and candidate-pair generation may be parallelized, although synchronization and pair deduplication must be handled carefully.

6. **Deterministic output**

   If deterministic collision-query ordering is required, candidate pairs should be sorted before narrow-phase processing.

### Proposed Implementation Phases

#### Phase 1: Uniform Spatial Hash

* Compute a swept AABB for every dynamic object.
* Map each swept AABB to integer grid coordinates.
* Insert object identifiers into hash-map buckets.
* Generate and deduplicate candidate pairs.
* Run the existing swept-AABB test and nonlinear CCD query on those pairs.

#### Phase 2: Memory and Allocation Optimization

* Reuse bucket vectors between simulation steps.
* Use compact object identifiers.
* Replace temporary hash sets where possible.
* Benchmark different cell sizes and hash functions.

#### Phase 3: Incremental Updates

* Retain the grid between simulation steps.
* Update only objects that move between cells.
* Avoid rebuilding the complete broad-phase structure when motion is limited.

#### Phase 4: Alternative Broad-Phase Comparison

Compare the uniform spatial hash against:

* Sweep and Prune
* Dynamic AABB trees
* Hierarchical spatial hashing
* Parry's existing broad-phase structures

The final implementation should be selected using representative benchmarks rather than theoretical complexity alone.

### Expected Outcome

This optimization would make the continuous collision checker suitable for substantially larger simulations.

The existing swept-AABB test would remain useful as a second broad-phase filter:

```text
Spatial partitioning
        │
        ▼
Candidate object pairs
        │
        ▼
Swept-AABB intersection test
        │
        ▼
Primitive fast path or compound BVH traversal
        │
        ▼
Continuous collision result
```

Combining spatial partitioning with the existing swept-AABB and primitive fast-path optimizations would reduce unnecessary nonlinear collision queries at multiple levels of the collision-detection pipeline.
