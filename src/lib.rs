//! rapier2d 物理引擎的 C FFI（外部函数接口）绑定。
//!
//! 提供一套扁平的 C API，用于：
//! - 创建和管理 2D 物理世界
//! - 添加/删除碰撞体（collider）
//! - 推进物理模拟（step）
//! - 查询交叉/重叠（intersection）
//! - 执行射线检测（raycast）
//!
//! 这个库的目的是让其他语言（如 C、C++、C#、Lua 等）能通过 C 接口调用 rapier2d 引擎。

// 允许在 unsafe 函数内部直接进行 unsafe 操作，而不需要再嵌套一层 unsafe 块。
// 这是 Rust 2024 edition 的写法，旧版本中 unsafe fn 内部本身就隐含 unsafe。
#![allow(unsafe_op_in_unsafe_fn)]

// rapier2d::parry 是 rapier 底层使用的几何/碰撞检测库，这里用于射线（Ray）等类型
use rapier2d::parry;
// rapier2d::prelude 包含了 rapier 物理引擎的所有常用类型和 trait
use rapier2d::prelude::*;
// c_char: C 语言的 char 类型（在 Rust 中的对应）
// CString: Rust 拥有的、以 null 结尾的 C 风格字符串
use std::ffi::{c_char, CString};

// ---------------------------------------------------------------------------
// 日志回调（Log callback）
// ---------------------------------------------------------------------------

/// C 函数指针类型，用于接收日志消息。
/// Option 包裹表示这个函数指针可以为 None（即 C 中的 NULL）。
/// `unsafe extern "C"` 表示这是一个 C 调用约定的函数指针，调用时需要 unsafe。
/// 参数 `*const c_char` 是一个指向 C 风格字符串的指针。
pub type RpLogCallback = Option<unsafe extern "C" fn(*const c_char)>;

/// 全局静态变量，存储日志回调函数。
/// `static mut` 是可变的全局变量，访问它需要 unsafe（因为多线程不安全）。
/// 初始值为 None，表示没有注册回调。
static mut LOG_CALLBACK: RpLogCallback = None;

/// 注册一个回调函数来接收本库的日志消息。
/// 传 NULL 可以禁用日志。
///
/// `#[unsafe(no_mangle)]` 告诉编译器不要修改这个函数的名字（name mangling），
/// 这样 C 代码才能通过函数名 `rp_register_log_callback` 找到它。
/// `pub extern "C"` 表示这是一个公开的、使用 C 调用约定的函数。
#[unsafe(no_mangle)]
pub extern "C" fn rp_register_log_callback(callback: RpLogCallback) {
    // 写入全局可变变量需要 unsafe
    unsafe { LOG_CALLBACK = callback };
}

/// 内部辅助函数：发送日志消息给 C 端注册的回调。
/// 如果没有注册回调，则什么都不做。
fn rp_log(msg: &str) {
    unsafe {
        // 检查是否注册了回调函数
        if let Some(cb) = LOG_CALLBACK {
            // CString::new 会在字符串末尾加上 '\0'，变成 C 风格字符串。
            // 如果 msg 中间包含 '\0' 会返回 Err，这里用 if let Ok 来忽略这种情况。
            if let Ok(cstr) = CString::new(msg) {
                // as_ptr() 获取指向底层 C 字符串的原始指针，传给 C 端回调
                cb(cstr.as_ptr());
            }
        }
    }
}

// ---------------------------------------------------------------------------
// FFI 安全的类型定义
// ---------------------------------------------------------------------------
// 这些类型需要跨语言传递，所以必须使用 `#[repr(C)]` 来保证内存布局与 C 一致。
// Rust 默认的内存布局可能会重新排列字段顺序，`#[repr(C)]` 禁止这种优化。

/// 2D 向量，包含 x 和 y 两个 f32 分量。
/// `#[repr(C)]` - 使用 C 语言的内存布局，确保跨语言兼容。
/// `#[derive(...)]` - 自动派生常用 trait：
///   Clone/Copy: 允许按值复制（因为只有两个 f32，复制很便宜）
///   Debug: 允许用 {:?} 格式化打印
///   Default: 提供默认值（x=0.0, y=0.0）
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct RpVec2 {
    pub x: f32,
    pub y: f32,
}

/// 不透明句柄（opaque handle），代表一个碰撞体（collider）。
/// 使用 `rp_handle_is_valid` 可以检查句柄是否有效。
///
/// rapier 内部使用"分代索引"（generational index）来管理对象：
/// - `id`: 在数组中的索引位置
/// - `generation`: 代数，用于检测该位置是否已被回收重用
///
/// 这种设计避免了悬空指针问题——如果一个碰撞体被删除，
/// 它的位置可能被新对象占用，但 generation 会不同，所以旧句柄会被识别为无效。
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RpHandle {
    pub id: u32,
    pub generation: u32,
}

impl RpHandle {
    /// 从 rapier 内部的 ColliderHandle 转换为 FFI 安全的 RpHandle
    fn from_collider(h: ColliderHandle) -> Self {
        // into_raw_parts() 拆分出底层的 (index, generation) 元组
        let (id, generation) = h.into_raw_parts();
        Self { id, generation }
    }

    /// 将 RpHandle 转换回 rapier 内部的 ColliderHandle
    fn to_collider(self) -> ColliderHandle {
        ColliderHandle::from_raw_parts(self.id, self.generation)
    }

    /// 创建一个"无效"句柄，用于表示操作失败或无结果。
    /// 使用 u32::MAX 作为哨兵值（sentinel value）。
    fn invalid() -> Self {
        Self {
            id: u32::MAX,
            generation: u32::MAX,
        }
    }
}

/// 射线检测（raycast）的结果。
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RpRayHit {
    /// 被击中的碰撞体的句柄
    pub handle: RpHandle,
    /// 撞击时间（time-of-impact），即射线方向上的距离。
    /// 实际撞击点 = 射线起点 + 方向 * toi
    pub toi: f32,
    /// 是否命中了碰撞体
    pub hit: bool,
}

/// 带法线和撞击点的射线检测结果（比 RpRayHit 信息更丰富）。
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RpRayHitWithNormal {
    /// 被击中的碰撞体的句柄
    pub handle: RpHandle,
    /// 撞击时间（射线方向上的距离）
    pub toi: f32,
    /// 撞击点的世界坐标
    pub point: RpVec2,
    /// 撞击表面的法线方向（单位向量，指向碰撞体外部）
    pub normal: RpVec2,
    /// 是否命中了碰撞体
    pub hit: bool,
}

/// 两个碰撞体之间的接触对（contact pair）信息。
/// 在物理模拟步进（step）之后可以查询到。
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RpContactPairInfo {
    /// 接触对中的第一个碰撞体
    pub collider1: RpHandle,
    /// 接触对中的第二个碰撞体
    pub collider2: RpHandle,
    /// 是否存在活跃的接触（即两个碰撞体真正接触在一起）
    pub has_any_active_contact: bool,
}

// ---------------------------------------------------------------------------
// 碰撞事件回调类型
// ---------------------------------------------------------------------------

/// 两个碰撞体开始碰撞时调用的回调函数类型。
/// 参数是两个发生碰撞的碰撞体句柄。
pub type RpCollisionStartCallback = Option<unsafe extern "C" fn(RpHandle, RpHandle)>;

/// 两个碰撞体停止碰撞时调用的回调函数类型。
pub type RpCollisionStopCallback = Option<unsafe extern "C" fn(RpHandle, RpHandle)>;

/// 内部结构体，存储碰撞事件的回调函数。
/// 实现了 rapier 的 `EventHandler` trait，这样物理引擎在检测到碰撞事件时会调用它。
struct WorldCollisionEvents {
    start_cb: RpCollisionStartCallback,
    stop_cb: RpCollisionStopCallback,
}

/// 为 WorldCollisionEvents 实现 rapier 的 EventHandler trait。
/// 这使得物理引擎在每次 step 时，碰撞状态发生变化时能通知我们。
impl EventHandler for WorldCollisionEvents {
    /// 处理碰撞事件（开始碰撞 / 停止碰撞）
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,       // 参数前加 _ 表示我们不使用它
        _colliders: &ColliderSet,
        event: CollisionEvent,         // 碰撞事件枚举：Started 或 Stopped
        _contact_pair: Option<&ContactPair>,
    ) {
        match event {
            // 两个碰撞体开始接触
            CollisionEvent::Started(h1, h2, _) => {
                if let Some(cb) = self.start_cb {
                    // 将 rapier 内部句柄转换为 FFI 句柄，然后调用 C 端回调
                    unsafe { cb(RpHandle::from_collider(h1), RpHandle::from_collider(h2)) };
                }
            }
            // 两个碰撞体停止接触
            CollisionEvent::Stopped(h1, h2, _) => {
                if let Some(cb) = self.stop_cb {
                    unsafe { cb(RpHandle::from_collider(h1), RpHandle::from_collider(h2)) };
                }
            }
        }
    }

    /// 处理接触力事件（当碰撞产生的力超过阈值时触发）。
    /// 这里我们不需要处理这个事件，所以函数体为空。
    /// 但因为 EventHandler trait 要求实现它，所以必须写上。
    fn handle_contact_force_event(
        &self,
        _dt: f32,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        _contact_pair: &ContactPair,
        _total_force_magnitude: f32,
    ) {
        // 空实现 - 不处理接触力事件
    }
}

// ---------------------------------------------------------------------------
// 物理世界（不透明结构体）
// ---------------------------------------------------------------------------
// "不透明"（opaque）意味着 C 端只持有指向它的指针，不知道内部结构。
// C 端通过 rp_world_create 获取指针，通过 rp_world_destroy 释放它。

/// 物理世界结构体，持有 rapier 引擎运行所需的全部状态。
///
/// rapier 的设计是"ECS 友好"的，各个组件分开存储，而不是一个大的 World 对象。
/// 所以我们需要自己把这些组件组合在一起。
pub struct RpWorld {
    /// 物理管线：负责执行每一步模拟的核心引擎
    pipeline: PhysicsPipeline,
    /// 重力向量，例如 (0.0, -9.81) 表示向下的重力
    gravity: Vector,
    /// 积分参数：控制模拟的时间步长（dt）等设置
    integration_params: IntegrationParameters,
    /// 岛管理器：将不相互作用的物体分组（"岛"），优化模拟性能
    islands: IslandManager,
    /// 宽相碰撞检测（Broad Phase）：使用 BVH（层次包围盒）快速排除不可能碰撞的物体对
    broad_phase: BroadPhaseBvh,
    /// 窄相碰撞检测（Narrow Phase）：对宽相筛选出的物体对进行精确的碰撞检测
    narrow_phase: NarrowPhase,
    /// 刚体集合：存储所有刚体（rigid body）
    bodies: RigidBodySet,
    /// 碰撞体集合：存储所有碰撞体（collider），碰撞体附着在刚体上
    colliders: ColliderSet,
    /// 冲量关节集合：存储关节约束（如铰链、弹簧等），本库暂未使用
    impulse_joints: ImpulseJointSet,
    /// 多体关节集合：存储多体关节，本库暂未使用
    multibody_joints: MultibodyJointSet,
    /// CCD（连续碰撞检测）求解器：防止高速物体穿过薄物体
    ccd_solver: CCDSolver,
    /// 碰撞事件处理器：在碰撞开始/结束时调用 C 端注册的回调
    event_handler: WorldCollisionEvents,
}

// ---------------------------------------------------------------------------
// 物理世界的生命周期管理（创建、销毁、推进模拟）
// ---------------------------------------------------------------------------

/// 创建一个新的物理世界。
/// - `gravity_x`, `gravity_y`: 重力向量（例如 0.0, -9.81 表示向下重力）
/// - `start_cb`: 碰撞开始时的回调函数（传 NULL 表示不需要）
/// - `stop_cb`: 碰撞结束时的回调函数（传 NULL 表示不需要）
/// - 返回值: 指向新创建的 RpWorld 的原始指针（C 端需保存此指针）
#[unsafe(no_mangle)]
pub extern "C" fn rp_world_create(
    gravity_x: f32,
    gravity_y: f32,
    start_cb: RpCollisionStartCallback,
    stop_cb: RpCollisionStopCallback,
) -> *mut RpWorld {
    // Box::new 在堆上分配内存并构造 RpWorld
    let world = Box::new(RpWorld {
        pipeline: PhysicsPipeline::new(),
        gravity: Vector::new(gravity_x, gravity_y),
        integration_params: IntegrationParameters::default(), // 默认 dt = 1/60 秒
        islands: IslandManager::new(),
        broad_phase: BroadPhaseBvh::new(),
        narrow_phase: NarrowPhase::new(),
        bodies: RigidBodySet::new(),
        colliders: ColliderSet::new(),
        impulse_joints: ImpulseJointSet::new(),
        multibody_joints: MultibodyJointSet::new(),
        ccd_solver: CCDSolver::new(),
        event_handler: WorldCollisionEvents { start_cb, stop_cb },
    });
    // Box::into_raw 将 Box 转换为原始指针，同时放弃 Rust 的自动释放管理。
    // 之后必须通过 rp_world_destroy 手动释放，否则会内存泄漏。
    Box::into_raw(world)
}

/// 销毁物理世界并释放所有关联内存。
/// 传入的 world 指针在调用后将变为无效，不可再使用。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_world_destroy(world: *mut RpWorld) {
    if !world.is_null() {
        // Box::from_raw 将原始指针重新包装为 Box。
        // drop() 显式销毁这个 Box，Rust 会自动释放其中所有资源。
        drop(Box::from_raw(world));
    }
}

/// 推进物理模拟一步（一个时间步长 dt）。
/// 每调用一次，物理世界前进 dt 秒（默认 1/60 秒）。
/// 在这一步中会进行：碰撞检测、约束求解、位置更新、事件触发等。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_world_step(world: *mut RpWorld) {
    if world.is_null() {
        return;
    }
    // 解引用原始指针得到可变引用
    let w = &mut *world;
    // pipeline.step() 是 rapier 的核心函数，执行完整的一步物理模拟
    w.pipeline.step(
        w.gravity,                // 重力
        &w.integration_params,    // 积分参数（dt 等）
        &mut w.islands,           // 岛管理器
        &mut w.broad_phase,       // 宽相碰撞检测
        &mut w.narrow_phase,      // 窄相碰撞检测
        &mut w.bodies,            // 刚体集合
        &mut w.colliders,         // 碰撞体集合
        &mut w.impulse_joints,    // 冲量关节
        &mut w.multibody_joints,  // 多体关节
        &mut w.ccd_solver,        // CCD 求解器
        &(),                      // 查询管线钩子（这里不使用，传空元组）
        &w.event_handler,         // 碰撞事件处理器
    );
}

/// 设置模拟的时间步长（dt）。默认值为 1/60 秒。
/// 更小的 dt 意味着更精确但更慢的模拟。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_world_set_timestep(world: *mut RpWorld, dt: f32) {
    if world.is_null() {
        return;
    }
    (*world).integration_params.dt = dt;
}

/// 设置物理世界的重力。
/// 例如 (0.0, -9.81) 模拟地球重力，(0.0, 0.0) 为零重力。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_world_set_gravity(world: *mut RpWorld, x: f32, y: f32) {
    if world.is_null() {
        return;
    }
    (*world).gravity = Vector::new(x, y);
}

/// 刷新空间查询所使用的宽相 BVH。
///
/// 新创建或者被 `rp_collider_set_position / set_rotation / set_pose` 移动过的碰撞体，
/// 在下一次 `rp_world_step` 被调用之前不会出现在 BVH 中，因此射线检测、形状投射、
/// 区域相交查询都不会命中它们。调用这个函数可以在不推进物理模拟的情况下，把刚体
/// 最新位姿同步到其 collider，并把每个启用中的 collider 的 AABB 推送进 BVH。
///
/// 典型用法：
///   rp_collider_create_*(...);
///   rp_world_update_query_pipeline(world);
///   rp_query_*(...);
///
/// 注意：此函数只刷新查询索引，不触发碰撞事件。碰撞事件仍然只在 `rp_world_step` 中触发。
/// 该函数的复杂度为 O(N)，N = 世界中 collider 总数，建议批量添加后再一次性调用。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_world_update_query_pipeline(world: *mut RpWorld) {
    if world.is_null() {
        return;
    }
    let w = &mut *world;

    // 第一步：把刚体最新位姿同步到其 collider。
    // rp_collider_set_position 等函数只改了父刚体的 pose，没改 collider.pos；
    // 而 BVH 更新是基于 collider.pos 计算 AABB 的。
    let bodies_ref = &w.bodies;
    let parent_updates: Vec<(ColliderHandle, Pose)> = w
        .colliders
        .iter()
        .filter_map(|(h, co)| {
            let parent = co.parent()?;
            let body = bodies_ref.get(parent)?;
            let pos_wrt_parent = *co.position_wrt_parent()?;
            Some((h, *body.position() * pos_wrt_parent))
        })
        .collect();
    for (h, new_pose) in parent_updates {
        if let Some(co) = w.colliders.get_mut(h) {
            co.set_position(new_pose);
        }
    }

    // 第二步：把每个 collider 的 AABB 推进 BVH。
    // 使用 `broad_phase.set_aabb` 而不是 `broad_phase.update`：
    // set_aabb 不会消费 `ColliderSet` 里的 modified_colliders 列表，因此后续
    // `rp_world_step` 仍然能正常识别这些 collider、生成碰撞事件。
    let handles: Vec<ColliderHandle> = w.colliders.iter().map(|(h, _)| h).collect();
    for h in handles {
        if let Some(co) = w.colliders.get(h) {
            if !co.is_enabled() {
                continue;
            }
            let aabb = co.compute_broad_phase_aabb(&w.integration_params, &w.bodies);
            w.broad_phase.set_aabb(&w.integration_params, h, aabb);
        }
    }
}

/// 返回当前世界中碰撞体的数量。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_world_collider_count(world: *const RpWorld) -> u32 {
    if world.is_null() {
        return 0;
    }
    // as u32: 将 usize 转换为 u32（C 端更容易使用的类型）
    (*world).colliders.len() as u32
}

// ---------------------------------------------------------------------------
// 内部辅助函数：创建刚体并将碰撞体附着上去
// ---------------------------------------------------------------------------
// 在 rapier 中，碰撞体（collider）必须附着在刚体（rigid body）上。
// `body_type` 参数控制刚体类型：
//   0 = Fixed（固定）：不受任何力影响，完全静止
//   1 = Dynamic（动态）：受重力、碰撞力、外力影响，自动移动
//   2 = KinematicPositionBased（运动学）：不受力影响，但移动时会推开动态物体

/// 将 C 端的 body_type 整数转换为 rapier 的 RigidBodyType 枚举。
/// 0 = Fixed, 1 = Dynamic, 2 = KinematicPositionBased，其他值默认为 Fixed。
fn body_type_from_i32(body_type: i32) -> RigidBodyType {
    match body_type {
        1 => RigidBodyType::Dynamic,
        2 => RigidBodyType::KinematicPositionBased,
        _ => RigidBodyType::Fixed,
    }
}

/// 创建一个刚体，将碰撞体附着上去，并返回碰撞体句柄。
/// - `builder`: 碰撞体构建器（已配置好形状）
/// - `x`, `y`: 世界坐标位置
/// - `angle`: 旋转角度（弧度）
/// - `collider_type`: 0 = 实体碰撞体（Solid），非0 = 传感器（Sensor）
/// - `body_type`: 0 = Fixed, 1 = Dynamic, 2 = KinematicPositionBased
/// - `group`: 碰撞组位掩码（32位），用于过滤哪些碰撞体之间可以交互
fn insert_collider(w: &mut RpWorld, builder: ColliderBuilder, x: f32, y: f32, angle: f32, collider_type: i32, body_type: i32, group: u32) -> RpHandle {
    let rb_type = body_type_from_i32(body_type);
    // 1. 创建刚体，设定位置和旋转
    let body = w.bodies.insert(
        RigidBodyBuilder::new(rb_type)
            .translation(Vector::new(x, y))
            .rotation(angle),
    );
    // 2. 配置碰撞体属性
    // 根据刚体类型选择需要检测的碰撞类型组合
    let collision_types = ActiveCollisionTypes::FIXED_FIXED
        | ActiveCollisionTypes::DYNAMIC_FIXED
        | ActiveCollisionTypes::DYNAMIC_DYNAMIC
        | ActiveCollisionTypes::DYNAMIC_KINEMATIC
        | ActiveCollisionTypes::KINEMATIC_FIXED
        | ActiveCollisionTypes::KINEMATIC_KINEMATIC;
    let collider = builder
        // sensor(true) 表示传感器模式：只检测重叠，不产生物理碰撞响应
        .sensor(collider_type != 0)
        // 开启所有碰撞类型组合的检测
        .active_collision_types(collision_types)
        // 开启碰撞事件，这样碰撞开始/结束时才会触发回调
        .active_events(ActiveEvents::COLLISION_EVENTS)
        // 设置碰撞组：
        // - memberships（成员）: 本碰撞体属于哪些组（group 位掩码）
        // - filter（过滤）: 本碰撞体可以与哪些组交互（这里是 ALL，即所有组）
        // - InteractionTestMode::And: 使用 AND 逻辑判断是否交互
        .collision_groups(InteractionGroups::new(group.into(), Group::ALL, InteractionTestMode::And))
        .build();
    // 3. 将碰撞体插入世界，并关联到父刚体
    // insert_with_parent 返回 ColliderHandle，我们将其转换为 RpHandle 返回给 C 端
    RpHandle::from_collider(w.colliders.insert_with_parent(collider, body, &mut w.bodies))
}

// ---------------------------------------------------------------------------
// 碰撞体创建
// ---------------------------------------------------------------------------
// 公共参数说明：
// `collider_type`: 0 = 实体（Solid，产生物理碰撞），非0 = 传感器（Sensor，只检测重叠）
// `body_type`: 0 = Fixed（固定），1 = Dynamic（动态），2 = Kinematic（运动学）
// `group`: 碰撞组成员位掩码（32位），控制哪些碰撞体之间会被检测

/// 添加一个圆形（球形）碰撞体。
/// - `radius`: 圆的半径
/// - 圆形不需要旋转角度（因为圆是旋转对称的），固定为 0.0
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_create_circle(
    world: *mut RpWorld,
    x: f32,
    y: f32,
    radius: f32,
    collider_type: i32,
    body_type: i32,
    group: u32,
) -> RpHandle {
    if world.is_null() {
        return RpHandle::invalid();
    }
    insert_collider(&mut *world, ColliderBuilder::ball(radius), x, y, 0.0, collider_type, body_type, group)
}

/// 添加一个矩形（长方体）碰撞体。
/// `half_width` 和 `half_height` 是半宽和半高。
/// 例如 half_width=2.0, half_height=1.0 创建一个 4x2 的矩形。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_create_rect(
    world: *mut RpWorld,
    x: f32,
    y: f32,
    angle: f32,
    half_width: f32,
    half_height: f32,
    collider_type: i32,
    body_type: i32,
    group: u32,
) -> RpHandle {
    if world.is_null() {
        return RpHandle::invalid();
    }
    insert_collider(&mut *world, ColliderBuilder::cuboid(half_width, half_height), x, y, angle, collider_type, body_type, group)
}

/// 添加一个三角形碰撞体。三个顶点坐标是局部空间（local space）的。
/// 碰撞体被放置在世界坐标 (x, y) 处，并应用指定的旋转角度。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_create_triangle(
    world: *mut RpWorld,
    x: f32,
    y: f32,
    angle: f32,
    a: RpVec2,
    b: RpVec2,
    c: RpVec2,
    collider_type: i32,
    body_type: i32,
    group: u32,
) -> RpHandle {
    if world.is_null() {
        return RpHandle::invalid();
    }
    let builder = ColliderBuilder::triangle(
        Vector::new(a.x, a.y),
        Vector::new(b.x, b.y),
        Vector::new(c.x, c.y),
    );
    insert_collider(&mut *world, builder, x, y, angle, collider_type, body_type, group)
}

/// 从顶点数组创建一个凸多边形碰撞体（计算凸包）。
/// - `vertices`: 指向 `count` 个 RpVec2 结构体的指针（局部空间坐标）
/// - `count`: 顶点数量，至少为 3
/// - 返回值: 如果凸包计算失败（例如顶点共线），返回无效句柄
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_create_convex(
    world: *mut RpWorld,
    x: f32,
    y: f32,
    angle: f32,
    vertices: *const RpVec2,
    count: u32,
    collider_type: i32,
    body_type: i32,
    group: u32,
) -> RpHandle {
    // 参数安全检查：世界指针非空、顶点指针非空、至少3个顶点
    if world.is_null() || vertices.is_null() || count < 3 {
        return RpHandle::invalid();
    }
    let w = &mut *world;
    // 从 C 端传入的原始指针构造 Rust 切片（slice），然后转换为 rapier 的 Vector 类型
    let pts: Vec<_> = std::slice::from_raw_parts(vertices, count as usize)
        .iter()
        .map(|v| Vector::new(v.x, v.y))
        .collect();
    // convex_hull 会计算这些点的凸包。如果点太少或共线则返回 None
    match ColliderBuilder::convex_hull(&pts) {
        Some(builder) => insert_collider(w, builder, x, y, angle, collider_type, body_type, group),
        None => {
            rp_log("rp_collider_create_convex: convex hull computation failed");
            RpHandle::invalid()
        }
    }
}

/// 添加一个胶囊体碰撞体。胶囊沿 Y 轴方向。
/// `half_height` 是中间矩形部分的半高（不含两端半圆）。
/// 胶囊体 = 矩形 + 上下两个半圆，总高度 = 2 * (half_height + radius)
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_create_capsule(
    world: *mut RpWorld,
    x: f32,
    y: f32,
    angle: f32,
    half_height: f32,
    radius: f32,
    collider_type: i32,
    body_type: i32,
    group: u32,
) -> RpHandle {
    if world.is_null() {
        return RpHandle::invalid();
    }
    insert_collider(&mut *world, ColliderBuilder::capsule_y(half_height, radius), x, y, angle, collider_type, body_type, group)
}

/// 添加一个线段碰撞体。
/// `a` 和 `b` 是线段两端点的局部空间坐标。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_create_segment(
    world: *mut RpWorld,
    x: f32,
    y: f32,
    angle: f32,
    a: RpVec2,
    b: RpVec2,
    collider_type: i32,
    body_type: i32,
    group: u32,
) -> RpHandle {
    if world.is_null() {
        return RpHandle::invalid();
    }
    let builder = ColliderBuilder::segment(
        Vector::new(a.x, a.y),
        Vector::new(b.x, b.y),
    );
    insert_collider(&mut *world, builder, x, y, angle, collider_type, body_type, group)
}

/// 从世界中移除一个碰撞体（同时移除其父固定刚体）。
/// 如果该碰撞体存在并被成功移除，返回 true。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_destroy(world: *mut RpWorld, handle: RpHandle) -> bool {
    if world.is_null() {
        return false;
    }
    let w = &mut *world;
    let ch = handle.to_collider();
    // 在移除碰撞体之前，先获取其父刚体句柄（移除后就拿不到了）
    let parent = w.colliders.get(ch).and_then(|c| c.parent());
    // 移除碰撞体。is_some() 表示确实存在并被移除了
    let removed = w.colliders
        .remove(ch, &mut w.islands, &mut w.bodies, true)
        .is_some();
    // 同时移除创建碰撞体时自动创建的固定刚体（清理干净）
    if let Some(body_handle) = parent {
        w.bodies.remove(
            body_handle,
            &mut w.islands,
            &mut w.colliders,
            &mut w.impulse_joints,
            &mut w.multibody_joints,
            true,
        );
    }
    removed
}

// ---------------------------------------------------------------------------
// 碰撞体属性的读取和设置
// ---------------------------------------------------------------------------

/// 设置碰撞体的世界坐标位置（实际上是移动其父刚体）。
/// 因为碰撞体附着在刚体上，移动刚体就等于移动碰撞体。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_set_position(
    world: *mut RpWorld,
    handle: RpHandle,
    x: f32,
    y: f32,
) {
    if world.is_null() {
        return;
    }
    let w = &mut *world;
    if let Some(co) = w.colliders.get(handle.to_collider()) {
        if let Some(parent) = co.parent() {
            if let Some(body) = w.bodies.get_mut(parent) {
                // 保留当前旋转角度，只改变位置
                let rot = body.rotation().angle();
                body.set_position(Pose::new(Vector::new(x, y), rot), true);
            }
        } else {
            // 独立碰撞体（没有父刚体的情况），直接设置碰撞体位置
            if let Some(co) = w.colliders.get_mut(handle.to_collider()) {
                co.set_translation(Vector::new(x, y));
            }
        }
    }
}

/// 设置碰撞体的世界空间旋转角度（弧度）。实际上是旋转其父刚体。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_set_rotation(
    world: *mut RpWorld,
    handle: RpHandle,
    angle: f32,
) {
    if world.is_null() {
        return;
    }
    let w = &mut *world;
    if let Some(co) = w.colliders.get(handle.to_collider()) {
        if let Some(parent) = co.parent() {
            if let Some(body) = w.bodies.get_mut(parent) {
                // 保留当前位置，只改变旋转角度
                let pos = body.translation();
                body.set_position(Pose::new(pos, angle), true);
            }
        } else {
            if let Some(co) = w.colliders.get_mut(handle.to_collider()) {
                co.set_rotation(Rotation::new(angle));
            }
        }
    }
}

/// 同时设置碰撞体的位置和旋转角度（比分别调用 set_position + set_rotation 更高效）。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_set_pose(
    world: *mut RpWorld,
    handle: RpHandle,
    x: f32,
    y: f32,
    angle: f32,
) {
    if world.is_null() {
        return;
    }
    let w = &mut *world;
    if let Some(co) = w.colliders.get(handle.to_collider()) {
        if let Some(parent) = co.parent() {
            if let Some(body) = w.bodies.get_mut(parent) {
                body.set_position(Pose::new(Vector::new(x, y), angle), true);
            }
        } else {
            if let Some(co) = w.colliders.get_mut(handle.to_collider()) {
                co.set_position(Pose::new(Vector::new(x, y), angle));
            }
        }
    }
}

/// 获取碰撞体的世界坐标位置。
/// 如果句柄无效，返回 (0, 0)。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_get_position(
    world: *const RpWorld,
    handle: RpHandle,
) -> RpVec2 {
    if world.is_null() {
        return RpVec2::default();
    }
    let w = &*world;
    if let Some(co) = w.colliders.get(handle.to_collider()) {
        // 从父刚体读取位置（set_position 后父刚体的值是最新的）
        if let Some(parent) = co.parent() {
            if let Some(body) = w.bodies.get(parent) {
                let t = body.translation();
                return RpVec2 { x: t.x, y: t.y };
            }
        }
        let t = co.position().translation;
        RpVec2 { x: t.x, y: t.y }
    } else {
        RpVec2::default()
    }
}

/// 获取碰撞体的世界空间旋转角度（弧度）。
/// 如果句柄无效，返回 0。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_get_rotation(
    world: *const RpWorld,
    handle: RpHandle,
) -> f32 {
    if world.is_null() {
        return 0.0;
    }
    let w = &*world;
    if let Some(co) = w.colliders.get(handle.to_collider()) {
        if let Some(parent) = co.parent() {
            if let Some(body) = w.bodies.get(parent) {
                return body.rotation().angle();
            }
        }
        co.position().rotation.angle()
    } else {
        0.0
    }
}

/// 返回碰撞体是否启用。禁用的碰撞体不参与碰撞检测和查询。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_is_enabled(
    world: *const RpWorld,
    handle: RpHandle,
) -> bool {
    if world.is_null() {
        return false;
    }
    (*world)
        .colliders
        .get(handle.to_collider())
        .map_or(false, |c| c.is_enabled())
}

/// 启用或禁用碰撞体。禁用的碰撞体将被排除在查询和模拟之外。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_set_enabled(
    world: *mut RpWorld,
    handle: RpHandle,
    enabled: bool,
) {
    if world.is_null() {
        return;
    }
    if let Some(co) = (*world).colliders.get_mut(handle.to_collider()) {
        co.set_enabled(enabled);
    }
}

/// 设置碰撞体是否为传感器模式。
/// - 传感器（Sensor）：只触发重叠事件，不产生物理碰撞响应（物体可以穿过）
/// - 实体（Solid）：产生真正的物理碰撞
/// collider_type: 0 = 实体（Solid），非0 = 传感器（Sensor）
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_set_type(
    world: *mut RpWorld,
    handle: RpHandle,
    collider_type: i32,
) {
    if world.is_null() {
        return;
    }
    if let Some(co) = (*world).colliders.get_mut(handle.to_collider()) {
        co.set_sensor(collider_type != 0);
    }
}

/// 返回碰撞体类型：0 = 实体（Solid），1 = 传感器（Sensor）。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_get_type(
    world: *const RpWorld,
    handle: RpHandle,
) -> i32 {
    if world.is_null() {
        return 0;
    }
    (*world)
        .colliders
        .get(handle.to_collider())
        .map_or(0, |c| if c.is_sensor() { 1 } else { 0 })
}

/// 获取碰撞体的碰撞组成员位掩码（32位）。
/// 碰撞组用于控制哪些碰撞体之间可以交互。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_get_group(
    world: *const RpWorld,
    handle: RpHandle,
) -> u32 {
    if world.is_null() {
        return 0;
    }
    (*world)
        .colliders
        .get(handle.to_collider())
        .map_or(0, |c| c.collision_groups().memberships.into())
}

/// 设置碰撞体的碰撞组成员位掩码（32位）。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_collider_set_group(
    world: *mut RpWorld,
    handle: RpHandle,
    group: u32,
) {
    if world.is_null() {
        return;
    }
    if let Some(co) = (*world).colliders.get_mut(handle.to_collider()) {
        co.set_collision_groups(InteractionGroups::new(group.into(), Group::ALL, InteractionTestMode::And));
    }
}

/// 检查句柄是否指向一个有效的碰撞体。
/// 如果碰撞体已被删除或句柄无效，返回 false。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_handle_is_valid(world: *const RpWorld, handle: RpHandle) -> bool {
    if world.is_null() {
        return false;
    }
    (*world).colliders.contains(handle.to_collider())
}

// ---------------------------------------------------------------------------
// 刚体属性（Dynamic / Kinematic 刚体的物理属性操作）
// ---------------------------------------------------------------------------
// 以下函数用于操作碰撞体的父刚体。
// 通过碰撞体句柄找到父刚体，然后操作刚体的物理属性。
// 对 Fixed 类型刚体调用这些函数不会产生效果（因为固定物体不受力影响）。

/// 内部辅助：通过碰撞体句柄获取其父刚体的可变引用
fn get_parent_body_mut<'a>(w: &'a mut RpWorld, handle: RpHandle) -> Option<&'a mut RigidBody> {
    let parent = w.colliders.get(handle.to_collider())?.parent()?;
    w.bodies.get_mut(parent)
}

/// 内部辅助：通过碰撞体句柄获取其父刚体的不可变引用
fn get_parent_body<'a>(w: &'a RpWorld, handle: RpHandle) -> Option<&'a RigidBody> {
    let parent = w.colliders.get(handle.to_collider())?.parent()?;
    w.bodies.get(parent)
}

/// 设置刚体类型。0 = Fixed, 1 = Dynamic, 2 = KinematicPositionBased。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_body_set_type(
    world: *mut RpWorld,
    handle: RpHandle,
    body_type: i32,
) {
    if world.is_null() { return; }
    if let Some(body) = get_parent_body_mut(&mut *world, handle) {
        body.set_body_type(body_type_from_i32(body_type), true);
    }
}

/// 获取刚体类型。0 = Fixed, 1 = Dynamic, 2 = Kinematic。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_body_get_type(
    world: *const RpWorld,
    handle: RpHandle,
) -> i32 {
    if world.is_null() { return 0; }
    get_parent_body(&*world, handle).map_or(0, |body| {
        match body.body_type() {
            RigidBodyType::Dynamic => 1,
            RigidBodyType::KinematicPositionBased | RigidBodyType::KinematicVelocityBased => 2,
            _ => 0,
        }
    })
}

/// 获取刚体的线速度。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_body_get_velocity(
    world: *const RpWorld,
    handle: RpHandle,
) -> RpVec2 {
    if world.is_null() { return RpVec2::default(); }
    get_parent_body(&*world, handle).map_or(RpVec2::default(), |body| {
        let v = body.linvel();
        RpVec2 { x: v.x, y: v.y }
    })
}

/// 设置刚体的线速度（仅对 Dynamic 有效）。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_body_set_velocity(
    world: *mut RpWorld,
    handle: RpHandle,
    vx: f32,
    vy: f32,
) {
    if world.is_null() { return; }
    if let Some(body) = get_parent_body_mut(&mut *world, handle) {
        body.set_linvel(Vector::new(vx, vy), true);
    }
}

/// 获取刚体的角速度（弧度/秒）。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_body_get_angular_velocity(
    world: *const RpWorld,
    handle: RpHandle,
) -> f32 {
    if world.is_null() { return 0.0; }
    get_parent_body(&*world, handle).map_or(0.0, |body| body.angvel())
}

/// 设置刚体的角速度（弧度/秒，仅对 Dynamic 有效）。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_body_set_angular_velocity(
    world: *mut RpWorld,
    handle: RpHandle,
    angvel: f32,
) {
    if world.is_null() { return; }
    if let Some(body) = get_parent_body_mut(&mut *world, handle) {
        body.set_angvel(angvel, true);
    }
}

/// 对刚体施加力（持续力，在下一次 step 中生效，仅对 Dynamic 有效）。
/// 力会在 step 结束后自动清零。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_body_apply_force(
    world: *mut RpWorld,
    handle: RpHandle,
    fx: f32,
    fy: f32,
) {
    if world.is_null() { return; }
    if let Some(body) = get_parent_body_mut(&mut *world, handle) {
        body.add_force(Vector::new(fx, fy), true);
    }
}

/// 对刚体施加冲量（瞬间推力，立即改变速度，仅对 Dynamic 有效）。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_body_apply_impulse(
    world: *mut RpWorld,
    handle: RpHandle,
    ix: f32,
    iy: f32,
) {
    if world.is_null() { return; }
    if let Some(body) = get_parent_body_mut(&mut *world, handle) {
        body.apply_impulse(Vector::new(ix, iy), true);
    }
}

/// 对刚体施加扭矩冲量（瞬间改变角速度，仅对 Dynamic 有效）。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_body_apply_torque_impulse(
    world: *mut RpWorld,
    handle: RpHandle,
    torque: f32,
) {
    if world.is_null() { return; }
    if let Some(body) = get_parent_body_mut(&mut *world, handle) {
        body.apply_torque_impulse(torque, true);
    }
}

/// 设置刚体的线性阻尼（用于模拟空气阻力，使物体逐渐减速）。
/// 0.0 = 无阻尼，值越大减速越快。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_body_set_linear_damping(
    world: *mut RpWorld,
    handle: RpHandle,
    damping: f32,
) {
    if world.is_null() { return; }
    if let Some(body) = get_parent_body_mut(&mut *world, handle) {
        body.set_linear_damping(damping);
    }
}

/// 设置刚体的角度阻尼（使旋转逐渐减速）。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_body_set_angular_damping(
    world: *mut RpWorld,
    handle: RpHandle,
    damping: f32,
) {
    if world.is_null() { return; }
    if let Some(body) = get_parent_body_mut(&mut *world, handle) {
        body.set_angular_damping(damping);
    }
}

/// 设置刚体的重力缩放系数。
/// 1.0 = 正常重力，0.0 = 无重力，2.0 = 双倍重力，-1.0 = 反重力。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_body_set_gravity_scale(
    world: *mut RpWorld,
    handle: RpHandle,
    scale: f32,
) {
    if world.is_null() { return; }
    if let Some(body) = get_parent_body_mut(&mut *world, handle) {
        body.set_gravity_scale(scale, true);
    }
}

/// 锁定刚体的旋转（防止碰撞导致物体旋转）。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_body_set_rotation_locked(
    world: *mut RpWorld,
    handle: RpHandle,
    locked: bool,
) {
    if world.is_null() { return; }
    if let Some(body) = get_parent_body_mut(&mut *world, handle) {
        body.lock_rotations(locked, true);
    }
}

// ---------------------------------------------------------------------------
// 射线检测（Raycasting）
// ---------------------------------------------------------------------------
// 射线检测是从一个起点沿指定方向发射一条射线，找出它击中的碰撞体。
// 常用于：枪击判定、视线检测、地面检测等。

/// 发射一条射线，返回第一个被击中的碰撞体。
/// - `origin_x/y`: 射线起点
/// - `dir_x/y`: 射线方向（不需要是单位向量）
/// - `max_toi`: 最大检测距离（沿射线方向）
/// - `group`: 过滤位掩码——只有碰撞组成员与此掩码有交集的碰撞体才会被检测
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_query_ray_cast(
    world: *const RpWorld,
    origin_x: f32,
    origin_y: f32,
    dir_x: f32,
    dir_y: f32,
    max_toi: f32,
    group: u32,
) -> RpRayHit {
    // 准备一个"未命中"的默认返回值
    let no_hit = RpRayHit {
        handle: RpHandle::invalid(),
        toi: 0.0,
        hit: false,
    };
    if world.is_null() {
        return no_hit;
    }
    let w = &*world;
    // 创建射线：起点 + 方向
    let ray = parry::query::Ray::new(
        Vector::new(origin_x, origin_y),
        Vector::new(dir_x, dir_y),
    );
    // 创建查询过滤器：
    // - memberships 设为 ALL（射线本身属于所有组）
    // - filter 设为 group（只与指定组的碰撞体交互）
    let filter = QueryFilter::default()
        .groups(InteractionGroups::new(Group::ALL, group.into(), InteractionTestMode::And));
    // 从宽相创建查询管线（query pipeline），用于执行空间查询
    let qp = w.broad_phase.as_query_pipeline(
        w.narrow_phase.query_dispatcher(),
        &w.bodies,
        &w.colliders,
        filter,
    );
    // cast_ray: 发射射线，返回最近的碰撞体和撞击时间
    // 第三个参数 true 表示将实体碰撞体视为实心（射线从内部不会命中）
    match qp.cast_ray(&ray, max_toi, true) {
        Some((handle, toi)) => RpRayHit {
            handle: RpHandle::from_collider(handle),
            toi,
            hit: true,
        },
        None => no_hit,
    }
}

/// 发射一条射线，返回第一个被击中的碰撞体，同时包含表面法线和撞击点信息。
/// 比 rp_query_ray_cast 提供更多信息，但开销稍大。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_query_ray_cast_with_normal(
    world: *const RpWorld,
    origin_x: f32,
    origin_y: f32,
    dir_x: f32,
    dir_y: f32,
    max_toi: f32,
    group: u32,
) -> RpRayHitWithNormal {
    let no_hit = RpRayHitWithNormal {
        handle: RpHandle::invalid(),
        toi: 0.0,
        point: RpVec2::default(),
        normal: RpVec2::default(),
        hit: false,
    };
    if world.is_null() {
        return no_hit;
    }
    let w = &*world;
    let ray = parry::query::Ray::new(
        Vector::new(origin_x, origin_y),
        Vector::new(dir_x, dir_y),
    );
    let filter = QueryFilter::default()
        .groups(InteractionGroups::new(Group::ALL, group.into(), InteractionTestMode::And));
    let qp = w.broad_phase.as_query_pipeline(
        w.narrow_phase.query_dispatcher(),
        &w.bodies,
        &w.colliders,
        filter,
    );
    // cast_ray_and_get_normal: 比 cast_ray 多返回法线信息
    match qp.cast_ray_and_get_normal(&ray, max_toi, true) {
        Some((handle, intersection)) => {
            // 计算撞击点：起点 + 方向 * 撞击时间
            let hit_point = ray.origin + ray.dir * intersection.time_of_impact;
            RpRayHitWithNormal {
                handle: RpHandle::from_collider(handle),
                toi: intersection.time_of_impact,
                point: RpVec2 {
                    x: hit_point.x,
                    y: hit_point.y,
                },
                normal: RpVec2 {
                    x: intersection.normal.x,
                    y: intersection.normal.y,
                },
                hit: true,
            }
        }
        None => no_hit,
    }
}

/// 发射一条射线，返回所有被穿过的碰撞体（不仅仅是第一个）。
/// - `out_handles`: C 端提供的输出缓冲区，用于写入命中的碰撞体句柄
/// - `max_results`: 缓冲区最大容量
/// - 返回值: 实际写入的结果数量
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_query_ray_cast_all(
    world: *const RpWorld,
    origin_x: f32,
    origin_y: f32,
    dir_x: f32,
    dir_y: f32,
    max_toi: f32,
    group: u32,
    out_handles: *mut RpHandle,
    max_results: u32,
) -> u32 {
    if world.is_null() || out_handles.is_null() {
        return 0;
    }
    let w = &*world;
    let ray = parry::query::Ray::new(
        Vector::new(origin_x, origin_y),
        Vector::new(dir_x, dir_y),
    );
    let filter = QueryFilter::default()
        .groups(InteractionGroups::new(Group::ALL, group.into(), InteractionTestMode::And));
    let qp = w.broad_phase.as_query_pipeline(
        w.narrow_phase.query_dispatcher(),
        &w.bodies,
        &w.colliders,
        filter,
    );
    // 将 C 端原始指针转换为 Rust 可变切片，方便按索引写入
    let buf = std::slice::from_raw_parts_mut(out_handles, max_results as usize);
    let mut count = 0u32;
    // intersect_ray 返回射线穿过的所有碰撞体的迭代器
    for (handle, _, _) in qp.intersect_ray(ray, max_toi, true) {
        if count >= max_results {
            break;
        }
        buf[count as usize] = RpHandle::from_collider(handle);
        count += 1;
    }
    count
}

/// 沿指定方向投射一个圆形（球形），返回第一个被碰到的碰撞体。
/// 这相当于"粗射线检测"（thick raycast）/ 扫掠测试（sweep test）。
/// 可以理解为：一个圆形沿着路径滑动，看它最先碰到什么。
/// - `radius`: 投射的圆形半径
/// - `exclude_handle`: 要从结果中排除的碰撞体句柄（常用于让物体避免检测到自身）。
///   传入 `RpHandle::invalid()`（id 和 generation 都是 u32::MAX）表示不排除任何 collider。
/// - 其他参数与射线检测类似
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_query_shape_cast_circle(
    world: *const RpWorld,
    origin_x: f32,
    origin_y: f32,
    dir_x: f32,
    dir_y: f32,
    max_toi: f32,
    radius: f32,
    group: u32,
    exclude_handle: RpHandle,
) -> RpRayHit {
    let no_hit = RpRayHit {
        handle: RpHandle::invalid(),
        toi: 0.0,
        hit: false,
    };
    if world.is_null() {
        return no_hit;
    }
    let w = &*world;
    let shape = Ball::new(radius);                      // 创建一个圆形
    let shape_pos = Pose::translation(origin_x, origin_y); // 圆形的初始位置
    let velocity = Vector::new(dir_x, dir_y);              // 圆形移动的方向和速度
    let mut filter = QueryFilter::default()
        .groups(InteractionGroups::new(Group::ALL, group.into(), InteractionTestMode::And));
    // RpHandle::invalid() 的两个字段都是 u32::MAX，只有两者都为 MAX 才视为无效哨兵
    if exclude_handle.id != u32::MAX || exclude_handle.generation != u32::MAX {
        filter = filter.exclude_collider(exclude_handle.to_collider());
    }
    let qp = w.broad_phase.as_query_pipeline(
        w.narrow_phase.query_dispatcher(),
        &w.bodies,
        &w.colliders,
        filter,
    );
    // cast_shape: 将一个形状沿指定方向投射，找出最先碰到的碰撞体
    // stop_at_penetration = false: 当形状已经穿透（重叠）时，若投射方向是分离方向，
    // 则忽略 toi=0 的命中，允许形状沿该方向移出。避免重叠后所有方向都报告碰撞导致卡死。
    let options = parry::query::ShapeCastOptions {
        stop_at_penetration: false,
        ..parry::query::ShapeCastOptions::with_max_time_of_impact(max_toi)
    };
    match qp.cast_shape(&shape_pos, velocity, &shape, options) {
        Some((handle, hit)) => RpRayHit {
            handle: RpHandle::from_collider(handle),
            toi: hit.time_of_impact,
            hit: true,
        },
        None => no_hit,
    }
}

/// 将一个圆形沿指定方向投射，返回第一个碰到的碰撞体，同时包含碰撞点和表面法线。
/// 比 rp_query_shape_cast_circle 提供更多信息（法线和碰撞点），适用于需要反弹方向等场景。
/// - `exclude_handle`: 要从结果中排除的碰撞体句柄（常用于让物体避免检测到自身）。
///   传入 `RpHandle::invalid()` 表示不排除任何 collider。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_query_shape_cast_circle_with_normal(
    world: *const RpWorld,
    origin_x: f32,
    origin_y: f32,
    dir_x: f32,
    dir_y: f32,
    max_toi: f32,
    radius: f32,
    group: u32,
    exclude_handle: RpHandle,
) -> RpRayHitWithNormal {
    let no_hit = RpRayHitWithNormal {
        handle: RpHandle::invalid(),
        toi: 0.0,
        point: RpVec2::default(),
        normal: RpVec2::default(),
        hit: false,
    };
    if world.is_null() {
        return no_hit;
    }
    let w = &*world;
    let shape = Ball::new(radius);
    let shape_pos = Pose::translation(origin_x, origin_y);
    let velocity = Vector::new(dir_x, dir_y);
    let mut filter = QueryFilter::default()
        .groups(InteractionGroups::new(Group::ALL, group.into(), InteractionTestMode::And));
    // RpHandle::invalid() 的两个字段都是 u32::MAX，只有两者都为 MAX 才视为无效哨兵
    if exclude_handle.id != u32::MAX || exclude_handle.generation != u32::MAX {
        filter = filter.exclude_collider(exclude_handle.to_collider());
    }
    let qp = w.broad_phase.as_query_pipeline(
        w.narrow_phase.query_dispatcher(),
        &w.bodies,
        &w.colliders,
        filter,
    );
    // stop_at_penetration = false: 同上，穿透时允许沿分离方向移出
    let options = parry::query::ShapeCastOptions {
        stop_at_penetration: false,
        ..parry::query::ShapeCastOptions::with_max_time_of_impact(max_toi)
    };
    match qp.cast_shape(&shape_pos, velocity, &shape, options) {
        Some((handle, hit)) => {
            // witness1 是投射形状（圆）上的碰撞点（局部坐标），加上起点得到世界坐标
            let point = hit.witness1 + Vector::new(origin_x, origin_y) + velocity * hit.time_of_impact;
            // normal1 是投射形状表面的法线方向（指向远离碰撞体的方向）
            RpRayHitWithNormal {
                handle: RpHandle::from_collider(handle),
                toi: hit.time_of_impact,
                point: RpVec2 { x: point.x, y: point.y },
                normal: RpVec2 { x: hit.normal1.x, y: hit.normal1.y },
                hit: true,
            }
        }
        None => no_hit,
    }
}

// ---------------------------------------------------------------------------
// 交叉/重叠查询（Intersection / Overlap queries）
// ---------------------------------------------------------------------------
// 这些查询用于检测碰撞体之间的重叠关系，不涉及射线。

/// 测试两个指定的碰撞体当前是否相交（重叠）。
/// 返回 true 表示它们有重叠区域。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_query_intersection_test(
    world: *const RpWorld,
    handle_a: RpHandle,
    handle_b: RpHandle,
) -> bool {
    if world.is_null() {
        return false;
    }
    let w = &*world;
    let a = handle_a.to_collider();
    let b = handle_b.to_collider();
    let co_a = match w.colliders.get(a) {
        Some(c) => c,
        None => return false,
    };
    let co_b = match w.colliders.get(b) {
        Some(c) => c,
        None => return false,
    };
    // inv_mul 计算从 A 坐标系到 B 坐标系的相对变换
    // 这是碰撞检测算法需要的：两个形状之间的相对位姿
    let pos12 = co_a.position().inv_mul(co_b.position());
    // intersection_test 执行精确的几何相交测试
    w.narrow_phase
        .query_dispatcher()
        .intersection_test(&pos12, co_a.shape(), co_b.shape())
        .unwrap_or(false) // 如果检测算法不支持该形状组合，返回 false
}

/// 测试一个点是否在任何碰撞体内部。返回第一个找到的碰撞体。
/// 如果没有碰撞体包含该点，返回无效句柄。
/// 常用于：鼠标点击拾取、区域检测等。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_query_point_intersect(
    world: *const RpWorld,
    x: f32,
    y: f32,
    group: u32,
) -> RpHandle {
    if world.is_null() {
        return RpHandle::invalid();
    }
    let w = &*world;
    let filter = QueryFilter::default()
        .groups(InteractionGroups::new(Group::ALL, group.into(), InteractionTestMode::And));
    let qp = w.broad_phase.as_query_pipeline(
        w.narrow_phase.query_dispatcher(),
        &w.bodies,
        &w.colliders,
        filter,
    );
    match qp.intersect_point(Vector::new(x, y)).next() {
        Some((handle, _)) => RpHandle::from_collider(handle),
        None => RpHandle::invalid(),
    }
}

/// 查找所有与指定圆形区域重叠的碰撞体。
/// - `x`, `y`: 圆心位置
/// - `radius`: 圆的半径
/// - `out_handles`: 输出缓冲区
/// - `max_results`: 缓冲区容量
/// - 返回值: 实际写入的结果数量
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_query_intersect_circle(
    world: *const RpWorld,
    x: f32,
    y: f32,
    radius: f32,
    group: u32,
    out_handles: *mut RpHandle,
    max_results: u32,
) -> u32 {
    if world.is_null() || out_handles.is_null() {
        return 0;
    }
    let w = &*world;
    let shape = Ball::new(radius);
    let shape_pos = Pose::translation(x, y);
    let filter = QueryFilter::default()
        .groups(InteractionGroups::new(Group::ALL, group.into(), InteractionTestMode::And));
    let qp = w.broad_phase.as_query_pipeline(
        w.narrow_phase.query_dispatcher(),
        &w.bodies,
        &w.colliders,
        filter,
    );
    let buf = std::slice::from_raw_parts_mut(out_handles, max_results as usize);
    let mut count = 0u32;
    for (handle, _) in qp.intersect_shape(shape_pos, &shape) {
        if count >= max_results {
            break;
        }
        buf[count as usize] = RpHandle::from_collider(handle);
        count += 1;
    }
    count
}

/// 查找所有与指定矩形区域重叠的碰撞体。
/// - `half_width`, `half_height`: 矩形的半宽和半高
/// - `angle`: 矩形的旋转角度（弧度）
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_query_intersect_rect(
    world: *const RpWorld,
    x: f32,
    y: f32,
    angle: f32,
    half_width: f32,
    half_height: f32,
    group: u32,
    out_handles: *mut RpHandle,
    max_results: u32,
) -> u32 {
    if world.is_null() || out_handles.is_null() {
        return 0;
    }
    let w = &*world;
    let shape = Cuboid::new(Vector::new(half_width, half_height));
    let shape_pos = Pose::new(Vector::new(x, y), angle);
    let filter = QueryFilter::default()
        .groups(InteractionGroups::new(Group::ALL, group.into(), InteractionTestMode::And));
    let qp = w.broad_phase.as_query_pipeline(
        w.narrow_phase.query_dispatcher(),
        &w.bodies,
        &w.colliders,
        filter,
    );
    let buf = std::slice::from_raw_parts_mut(out_handles, max_results as usize);
    let mut count = 0u32;
    for (handle, _) in qp.intersect_shape(shape_pos, &shape) {
        if count >= max_results {
            break;
        }
        buf[count as usize] = RpHandle::from_collider(handle);
        count += 1;
    }
    count
}

/// 查找所有与指定胶囊体区域重叠的碰撞体。
/// - `half_height`: 胶囊中间矩形部分的半高
/// - `radius`: 胶囊两端半圆的半径
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_query_intersect_capsule(
    world: *const RpWorld,
    x: f32,
    y: f32,
    angle: f32,
    half_height: f32,
    radius: f32,
    group: u32,
    out_handles: *mut RpHandle,
    max_results: u32,
) -> u32 {
    if world.is_null() || out_handles.is_null() {
        return 0;
    }
    let w = &*world;
    let shape = Capsule::new_y(half_height, radius);
    let shape_pos = Pose::new(Vector::new(x, y), angle);
    let filter = QueryFilter::default()
        .groups(InteractionGroups::new(Group::ALL, group.into(), InteractionTestMode::And));
    let qp = w.broad_phase.as_query_pipeline(
        w.narrow_phase.query_dispatcher(),
        &w.bodies,
        &w.colliders,
        filter,
    );
    let buf = std::slice::from_raw_parts_mut(out_handles, max_results as usize);
    let mut count = 0u32;
    for (handle, _) in qp.intersect_shape(shape_pos, &shape) {
        if count >= max_results {
            break;
        }
        buf[count as usize] = RpHandle::from_collider(handle);
        count += 1;
    }
    count
}

// ---------------------------------------------------------------------------
// 在指定圆内寻找空旷点
// ---------------------------------------------------------------------------

/// 简单的确定性伪随机数生成器（xorshift32）。
/// 用于在搜索圆内生成候选点，保持与 `enhanced-determinism` 特性一致。
fn xorshift32(state: &mut u32) -> u32 {
    let mut x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    x
}

/// 将 xorshift32 的输出映射到 [0.0, 1.0) 区间的 f32。
fn rand_f32(state: &mut u32) -> f32 {
    (xorshift32(state) & 0x00FF_FFFF) as f32 / 16_777_216.0
}

/// 在指定圆形区域内寻找一个"空旷"的点，该点周围 `r2` 半径内没有指定类型的碰撞体。
///
/// 算法：在搜索圆 (x, y, r) 内采样候选点，对每个候选点做半径 `r2` 的圆形区域查询，
/// 如果查询结果中没有匹配 `ignore_collider_types` 的碰撞体，则返回该点。
///
/// - `x`, `y`: 搜索圆心
/// - `r`: 搜索半径
/// - `r2`: 清空半径（该点周围多大范围内不能有碰撞体）
/// - `ignore_collider_types`: 位掩码，指定要避开的碰撞体类型
///   - bit 0 (0x1): 避开实体碰撞体（Solid, collider_type == 0）
///   - bit 1 (0x2): 避开传感器碰撞体（Sensor, collider_type != 0）
///   - 0x3: 避开所有碰撞体
/// - `group`: 碰撞组过滤位掩码
/// - `mode`: 采样模式
///   - 0 = 均匀随机采样（搜索圆内各处概率相同）
///   - 1 = 优先靠近中心（先尝试中心点，然后逐渐向外扩展）
/// - `max_attempts`: 最大尝试次数
/// - `out_found`: 输出是否成功找到有效点（true = 找到，false = 未找到）
/// - 返回值: 找到的空旷点坐标；如果未找到，返回 (0, 0)
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_query_find_clear_point(
    world: *const RpWorld,
    x: f32,
    y: f32,
    r: f32,
    r2: f32,
    ignore_collider_types: u32,
    group: u32,
    mode: i32,
    max_attempts: u32,
    out_found: *mut bool,
) -> RpVec2 {
    if world.is_null() {
        if !out_found.is_null() {
            *out_found = false;
        }
        return RpVec2::default();
    }
    let w = &*world;

    // 用位置和半径作为种子，保证确定性
    let seed = (x.to_bits() ^ y.to_bits() ^ r.to_bits()).wrapping_add(1);
    let mut rng_state = if seed == 0 { 1u32 } else { seed };

    let filter = QueryFilter::default()
        .groups(InteractionGroups::new(Group::ALL, group.into(), InteractionTestMode::And));

    let shape = Ball::new(r2);

    for i in 0..max_attempts {
        let (px, py) = match mode {
            // mode 1: 优先靠近中心
            // 第一次尝试中心点，之后按 i/max_attempts 比例逐渐向外扩展，配合随机角度
            1 => {
                if i == 0 {
                    (x, y)
                } else {
                    let angle = rand_f32(&mut rng_state) * std::f32::consts::TAU;
                    let t = i as f32 / max_attempts as f32;
                    let dist = r * t;
                    (x + dist * angle.cos(), y + dist * angle.sin())
                }
            }
            // mode 0 (默认): 均匀随机采样（极坐标法，sqrt 保证面积均匀）
            _ => {
                let angle = rand_f32(&mut rng_state) * std::f32::consts::TAU;
                let dist = r * rand_f32(&mut rng_state).sqrt();
                (x + dist * angle.cos(), y + dist * angle.sin())
            }
        };

        // 在候选点周围 r2 范围内查找碰撞体
        let shape_pos = Pose::translation(px, py);
        let qp = w.broad_phase.as_query_pipeline(
            w.narrow_phase.query_dispatcher(),
            &w.bodies,
            &w.colliders,
            filter,
        );

        let mut found_blocking = false;
        for (handle, _) in qp.intersect_shape(shape_pos, &shape) {
            // 检查碰撞体类型是否在避开列表中
            if let Some(co) = w.colliders.get(handle) {
                let is_sensor = co.is_sensor();
                // bit 0: 避开 Solid (非 sensor)
                // bit 1: 避开 Sensor
                if (!is_sensor && (ignore_collider_types & 0x1) != 0)
                    || (is_sensor && (ignore_collider_types & 0x2) != 0)
                {
                    found_blocking = true;
                    break;
                }
            }
        }

        if !found_blocking {
            if !out_found.is_null() {
                *out_found = true;
            }
            return RpVec2 { x: px, y: py };
        }
    }

    // 所有尝试都失败了
    if !out_found.is_null() {
        *out_found = false;
    }
    RpVec2::default()
}

/// 在指定长方形区域内寻找一个"空旷"的点，该点周围 `r2` 半径内没有指定类型的碰撞体。
///
/// - `rect_x`, `rect_y`: 搜索矩形左下角坐标
/// - `rect_w`, `rect_h`: 搜索矩形的宽度和高度
/// - `r2`: 清空半径（候选点周围多大范围内不能有碰撞体）
/// - `ignore_collider_types`: 位掩码，指定要避开的碰撞体类型
///   - bit 0 (0x1): 避开实体碰撞体（Solid）
///   - bit 1 (0x2): 避开传感器碰撞体（Sensor）
/// - `group`: 碰撞组过滤位掩码
/// - `mode`: 采样模式
///   - 0 = 均匀随机采样
///   - 1 = 优先靠近中心（先尝试矩形中心，再随机扩展）
/// - `max_attempts`: 最大尝试次数
/// - `out_found`: 输出是否成功找到有效点
/// - 返回值: 找到的空旷点坐标；如果未找到，返回 (0, 0)
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_query_find_clear_point_in_rect(
    world: *const RpWorld,
    rect_x: f32,
    rect_y: f32,
    rect_w: f32,
    rect_h: f32,
    r2: f32,
    ignore_collider_types: u32,
    group: u32,
    mode: i32,
    max_attempts: u32,
    out_found: *mut bool,
) -> RpVec2 {
    if world.is_null() {
        if !out_found.is_null() {
            *out_found = false;
        }
        return RpVec2::default();
    }
    let w = &*world;

    let seed = (rect_x.to_bits() ^ rect_y.to_bits() ^ rect_w.to_bits() ^ rect_h.to_bits())
        .wrapping_add(1);
    let mut rng_state = if seed == 0 { 1u32 } else { seed };

    let filter = QueryFilter::default()
        .groups(InteractionGroups::new(Group::ALL, group.into(), InteractionTestMode::And));

    let shape = Ball::new(r2);

    let cx = rect_x + rect_w * 0.5;
    let cy = rect_y + rect_h * 0.5;

    for i in 0..max_attempts {
        let (px, py) = match mode {
            // mode 1: 优先靠近中心，逐渐向外扩展
            1 => {
                if i == 0 {
                    (cx, cy)
                } else {
                    let t = i as f32 / max_attempts as f32;
                    let ox = (rand_f32(&mut rng_state) - 0.5) * rect_w * t;
                    let oy = (rand_f32(&mut rng_state) - 0.5) * rect_h * t;
                    (cx + ox, cy + oy)
                }
            }
            // mode 0 (默认): 矩形内均匀随机采样
            _ => {
                let px = rect_x + rand_f32(&mut rng_state) * rect_w;
                let py = rect_y + rand_f32(&mut rng_state) * rect_h;
                (px, py)
            }
        };

        let shape_pos = Pose::translation(px, py);
        let qp = w.broad_phase.as_query_pipeline(
            w.narrow_phase.query_dispatcher(),
            &w.bodies,
            &w.colliders,
            filter,
        );

        let mut found_blocking = false;
        for (handle, _) in qp.intersect_shape(shape_pos, &shape) {
            if let Some(co) = w.colliders.get(handle) {
                let is_sensor = co.is_sensor();
                if (!is_sensor && (ignore_collider_types & 0x1) != 0)
                    || (is_sensor && (ignore_collider_types & 0x2) != 0)
                {
                    found_blocking = true;
                    break;
                }
            }
        }

        if !found_blocking {
            if !out_found.is_null() {
                *out_found = true;
            }
            return RpVec2 { x: px, y: py };
        }
    }

    if !out_found.is_null() {
        *out_found = false;
    }
    RpVec2::default()
}

// ---------------------------------------------------------------------------
// 接触对遍历（在 step 之后使用）
// ---------------------------------------------------------------------------
// 在每次 rp_world_step 之后，可以查询哪些碰撞体之间产生了接触。
// 这对于游戏逻辑非常有用，例如检测角色是否站在地面上。

/// 返回上一次 step 后的接触对数量。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_world_contact_pair_count(world: *const RpWorld) -> u32 {
    if world.is_null() {
        return 0;
    }
    (*world).narrow_phase.contact_pairs().count() as u32
}

/// 将接触对信息复制到 `out_pairs` 缓冲区（最多 `max_pairs` 个）。
/// 返回实际写入的数量。
/// C 端需要预先分配足够大的缓冲区（可以先调用 rp_world_contact_pair_count 获取数量）。
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rp_world_get_contact_pairs(
    world: *const RpWorld,
    out_pairs: *mut RpContactPairInfo,
    max_pairs: u32,
) -> u32 {
    if world.is_null() || out_pairs.is_null() {
        return 0;
    }
    let w = &*world;
    let buf = std::slice::from_raw_parts_mut(out_pairs, max_pairs as usize);
    let mut count = 0u32;
    for pair in w.narrow_phase.contact_pairs() {
        if count >= max_pairs {
            break;
        }
        buf[count as usize] = RpContactPairInfo {
            collider1: RpHandle::from_collider(pair.collider1),
            collider2: RpHandle::from_collider(pair.collider2),
            has_any_active_contact: pair.has_any_active_contact(),
        };
        count += 1;
    }
    count
}
