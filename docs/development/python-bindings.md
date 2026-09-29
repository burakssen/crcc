# Python bindings development

Maturin builds `crcc._core` from the Rust crate with `python_bindings` and all three backends. `python_bindings` enables Rayon and requires a backend. PyO3 uses `abi3-py310`; Python source is under `python/crcc/`.

## Expose a type or operation

1. Implement/document the Rust domain/query contract first.
2. Add the PyO3 wrapper or method in `src/python/`. Follow existing `Arc` ownership, pose extraction, backend resolution and `CrccError -> ValueError` conversion.
3. Register native classes/functions/submodules in the corresponding binding module and `src/python/mod.rs`. Python native backend naming is `CollisionBackend`, not the Rust enum name.
4. Update native `_core/*.pyi` with the **native** signature. Do not copy the public facade builder into the native builder stub.
5. Add public wrapper/re-export and facade `.pyi` entries, including root `__all__` where applicable. Keep CommonRoad adapters in Python, separate from the core API.
6. If adding batches, deliberately define extraction, prepared/raw dispatch, GIL release and error collection. Current Python batches return a complete list or raise; Rust retains per-entry errors.
7. Rebuild with Maturin; test runtime keyword signatures, imports, errors, compatibility warnings and packaged-wheel behavior. Run Pyright and executable documentation checks.

## Reference strategy

The site contains a handwritten complete public reference and behavioral guides. Stub declarations are checked against a rebuilt extension for native signatures and documented exports; Rust exact signatures come from generated Rustdoc. This avoids making the Zensical build import an extension or scenario data. No generated Python reference from private `_core` modules should replace facade signatures.

Bundled stubs support repository checks, but the package currently lacks a `py.typed` marker. Do not assume downstream type-checker discovery is established merely because Pyright passes inside this repository. Benchmark helpers under `_core.benchmark` are internal and should not be root-exported as application APIs.
