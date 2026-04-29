# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
# Build library (also regenerates rapier2d_ffi.h via cbindgen)
cargo build --release

# Compile C tests (Linux)
cc -o test/test_rapier2d test/test_rapier2d.c \
  -I. -Ltarget/release -lrapier2d_ffi -lm -lpthread -ldl

# Run C tests (Linux)
LD_LIBRARY_PATH=target/release ./test/test_rapier2d

# Compile C tests (macOS)
cc -o test/test_rapier2d test/test_rapier2d.c \
  -I. -Ltarget/release -lrapier2d_ffi -framework Security -framework CoreFoundation -lm
DYLD_LIBRARY_PATH=target/release ./test/test_rapier2d

# Cross-platform dist builds
./build.sh linux     # Linux x86_64
./build.sh mac       # macOS Universal Binary (requires macOS + lipo)
./build.sh windows   # Windows x86_64 (requires mingw-w64)
```

There is no `cargo test` — all tests live in `test/test_rapier2d.c` and are compiled and run as a C binary.

## Architecture

**Single source file:** All FFI implementation is in `src/lib.rs`. There are no modules.

**Header is auto-generated:** `rapier2d_ffi.h` is produced by cbindgen at build time via `build.rs`. Never edit it directly — all changes go in `src/lib.rs`.

**RpWorld bundles all rapier components:** rapier2d uses an ECS-style split (pipeline, island manager, broad phase, narrow phase, body set, collider set, joint sets, CCD solver). `RpWorld` owns all of them in one struct, allocated on the heap and handed to C as an opaque pointer via `Box::into_raw` / `Box::from_raw`.

**1:1 collider-to-rigidbody:** Every `rp_collider_create_*` function creates a dedicated `RigidBody` and attaches the collider to it. `RpHandle` wraps `ColliderHandle` (a generational index `{id, generation}`). Body lookups go through `collider → parent body handle → body`.

**Spatial queries reuse BroadPhaseBvh:** Area intersection queries (`intersect_circle`, `intersect_rect`, etc.) and `find_clear_point*` call `BroadPhaseBvh::as_query_pipeline(...)` rather than running a full physics step. Shape positions are expressed as `Pose::translation(x, y)`.

**Deterministic sampling:** `rand_f32` is a simple LCG seeded from the input coordinates via XOR + `wrapping_add(1)`. The same call arguments always produce the same sequence, which is the guarantee behind `rp_query_find_clear_point` and `rp_query_find_clear_point_in_rect`.

**Collision groups:** Every collider has a membership bitmask. Queries pass a filter bitmask. A collider is included when `(membership & filter) != 0`. The rapier `InteractionGroups` constructor maps directly to this: `InteractionGroups::new(Group::ALL, filter.into(), InteractionTestMode::And)`.

**NULL safety contract:** Every exported function checks `world.is_null()` at entry and returns a safe zero/false/empty result. This must be maintained for any new functions.

## Adding a new API function

1. Implement in `src/lib.rs` with `#[unsafe(no_mangle)] pub unsafe extern "C" fn rp_...`.
2. Run `cargo build` — cbindgen regenerates `rapier2d_ffi.h` automatically.
3. Add test cases in `test/test_rapier2d.c`: determinism, bounds, NULL safety, and a `RUN_TEST(...)` registration in `main`.
