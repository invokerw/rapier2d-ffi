#ifndef RAPIER2D_FFI_H
#define RAPIER2D_FFI_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * 物理世界结构体，持有 rapier 引擎运行所需的全部状态。
 *
 * rapier 的设计是"ECS 友好"的，各个组件分开存储，而不是一个大的 World 对象。
 * 所以我们需要自己把这些组件组合在一起。
 */
typedef struct RpWorld RpWorld;

/**
 * C 函数指针类型，用于接收日志消息。
 * Option 包裹表示这个函数指针可以为 None（即 C 中的 NULL）。
 * `unsafe extern "C"` 表示这是一个 C 调用约定的函数指针，调用时需要 unsafe。
 * 参数 `*const c_char` 是一个指向 C 风格字符串的指针。
 */
typedef void (*RpLogCallback)(const char*);

/**
 * 不透明句柄（opaque handle），代表一个碰撞体（collider）。
 * 使用 `rp_handle_is_valid` 可以检查句柄是否有效。
 *
 * rapier 内部使用"分代索引"（generational index）来管理对象：
 * - `id`: 在数组中的索引位置
 * - `generation`: 代数，用于检测该位置是否已被回收重用
 *
 * 这种设计避免了悬空指针问题——如果一个碰撞体被删除，
 * 它的位置可能被新对象占用，但 generation 会不同，所以旧句柄会被识别为无效。
 */
typedef struct RpHandle {
    uint32_t id;
    uint32_t generation;
} RpHandle;

/**
 * 两个碰撞体开始碰撞时调用的回调函数类型。
 * 参数是两个发生碰撞的碰撞体句柄。
 */
typedef void (*RpCollisionStartCallback)(struct RpHandle, struct RpHandle);

/**
 * 两个碰撞体停止碰撞时调用的回调函数类型。
 */
typedef void (*RpCollisionStopCallback)(struct RpHandle, struct RpHandle);

/**
 * 2D 向量，包含 x 和 y 两个 f32 分量。
 * `#[repr(C)]` - 使用 C 语言的内存布局，确保跨语言兼容。
 * `#[derive(...)]` - 自动派生常用 trait：
 *   Clone/Copy: 允许按值复制（因为只有两个 f32，复制很便宜）
 *   Debug: 允许用 {:?} 格式化打印
 *   Default: 提供默认值（x=0.0, y=0.0）
 */
typedef struct RpVec2 {
    float x;
    float y;
} RpVec2;

/**
 * 射线检测（raycast）的结果。
 */
typedef struct RpRayHit {
    /**
     * 被击中的碰撞体的句柄
     */
    struct RpHandle handle;
    /**
     * 撞击时间（time-of-impact），即射线方向上的距离。
     * 实际撞击点 = 射线起点 + 方向 * toi
     */
    float toi;
    /**
     * 是否命中了碰撞体
     */
    bool hit;
} RpRayHit;

/**
 * 带法线和撞击点的射线检测结果（比 RpRayHit 信息更丰富）。
 */
typedef struct RpRayHitWithNormal {
    /**
     * 被击中的碰撞体的句柄
     */
    struct RpHandle handle;
    /**
     * 撞击时间（射线方向上的距离）
     */
    float toi;
    /**
     * 撞击点的世界坐标
     */
    struct RpVec2 point;
    /**
     * 撞击表面的法线方向（单位向量，指向碰撞体外部）
     */
    struct RpVec2 normal;
    /**
     * 是否命中了碰撞体
     */
    bool hit;
} RpRayHitWithNormal;

/**
 * 两个碰撞体之间的接触对（contact pair）信息。
 * 在物理模拟步进（step）之后可以查询到。
 */
typedef struct RpContactPairInfo {
    /**
     * 接触对中的第一个碰撞体
     */
    struct RpHandle collider1;
    /**
     * 接触对中的第二个碰撞体
     */
    struct RpHandle collider2;
    /**
     * 是否存在活跃的接触（即两个碰撞体真正接触在一起）
     */
    bool has_any_active_contact;
} RpContactPairInfo;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * 注册一个回调函数来接收本库的日志消息。
 * 传 NULL 可以禁用日志。
 *
 * `#[unsafe(no_mangle)]` 告诉编译器不要修改这个函数的名字（name mangling），
 * 这样 C 代码才能通过函数名 `rp_register_log_callback` 找到它。
 * `pub extern "C"` 表示这是一个公开的、使用 C 调用约定的函数。
 */
void rp_register_log_callback(RpLogCallback callback);

/**
 * 创建一个新的物理世界。
 * - `gravity_x`, `gravity_y`: 重力向量（例如 0.0, -9.81 表示向下重力）
 * - `start_cb`: 碰撞开始时的回调函数（传 NULL 表示不需要）
 * - `stop_cb`: 碰撞结束时的回调函数（传 NULL 表示不需要）
 * - 返回值: 指向新创建的 RpWorld 的原始指针（C 端需保存此指针）
 */
struct RpWorld *rp_world_create(float gravity_x,
                                float gravity_y,
                                RpCollisionStartCallback start_cb,
                                RpCollisionStopCallback stop_cb);

/**
 * 销毁物理世界并释放所有关联内存。
 * 传入的 world 指针在调用后将变为无效，不可再使用。
 */
void rp_world_destroy(struct RpWorld *world);

/**
 * 推进物理模拟一步（一个时间步长 dt）。
 * 每调用一次，物理世界前进 dt 秒（默认 1/60 秒）。
 * 在这一步中会进行：碰撞检测、约束求解、位置更新、事件触发等。
 */
void rp_world_step(struct RpWorld *world);

/**
 * 设置模拟的时间步长（dt）。默认值为 1/60 秒。
 * 更小的 dt 意味着更精确但更慢的模拟。
 */
void rp_world_set_timestep(struct RpWorld *world, float dt);

/**
 * 设置物理世界的重力。
 * 例如 (0.0, -9.81) 模拟地球重力，(0.0, 0.0) 为零重力。
 */
void rp_world_set_gravity(struct RpWorld *world, float x, float y);

/**
 * 返回当前世界中碰撞体的数量。
 */
uint32_t rp_world_collider_count(const struct RpWorld *world);

/**
 * 添加一个圆形（球形）碰撞体。
 * - `radius`: 圆的半径
 * - 圆形不需要旋转角度（因为圆是旋转对称的），固定为 0.0
 */
struct RpHandle rp_collider_create_circle(struct RpWorld *world,
                                          float x,
                                          float y,
                                          float radius,
                                          int32_t collider_type,
                                          int32_t body_type,
                                          uint32_t group);

/**
 * 添加一个矩形（长方体）碰撞体。
 * `half_width` 和 `half_height` 是半宽和半高。
 * 例如 half_width=2.0, half_height=1.0 创建一个 4x2 的矩形。
 */
struct RpHandle rp_collider_create_rect(struct RpWorld *world,
                                        float x,
                                        float y,
                                        float angle,
                                        float half_width,
                                        float half_height,
                                        int32_t collider_type,
                                        int32_t body_type,
                                        uint32_t group);

/**
 * 添加一个三角形碰撞体。三个顶点坐标是局部空间（local space）的。
 * 碰撞体被放置在世界坐标 (x, y) 处，并应用指定的旋转角度。
 */
struct RpHandle rp_collider_create_triangle(struct RpWorld *world,
                                            float x,
                                            float y,
                                            float angle,
                                            struct RpVec2 a,
                                            struct RpVec2 b,
                                            struct RpVec2 c,
                                            int32_t collider_type,
                                            int32_t body_type,
                                            uint32_t group);

/**
 * 从顶点数组创建一个凸多边形碰撞体（计算凸包）。
 * - `vertices`: 指向 `count` 个 RpVec2 结构体的指针（局部空间坐标）
 * - `count`: 顶点数量，至少为 3
 * - 返回值: 如果凸包计算失败（例如顶点共线），返回无效句柄
 */
struct RpHandle rp_collider_create_convex(struct RpWorld *world,
                                          float x,
                                          float y,
                                          float angle,
                                          const struct RpVec2 *vertices,
                                          uint32_t count,
                                          int32_t collider_type,
                                          int32_t body_type,
                                          uint32_t group);

/**
 * 添加一个胶囊体碰撞体。胶囊沿 Y 轴方向。
 * `half_height` 是中间矩形部分的半高（不含两端半圆）。
 * 胶囊体 = 矩形 + 上下两个半圆，总高度 = 2 * (half_height + radius)
 */
struct RpHandle rp_collider_create_capsule(struct RpWorld *world,
                                           float x,
                                           float y,
                                           float angle,
                                           float half_height,
                                           float radius,
                                           int32_t collider_type,
                                           int32_t body_type,
                                           uint32_t group);

/**
 * 添加一个线段碰撞体。
 * `a` 和 `b` 是线段两端点的局部空间坐标。
 */
struct RpHandle rp_collider_create_segment(struct RpWorld *world,
                                           float x,
                                           float y,
                                           float angle,
                                           struct RpVec2 a,
                                           struct RpVec2 b,
                                           int32_t collider_type,
                                           int32_t body_type,
                                           uint32_t group);

/**
 * 从世界中移除一个碰撞体（同时移除其父固定刚体）。
 * 如果该碰撞体存在并被成功移除，返回 true。
 */
bool rp_collider_destroy(struct RpWorld *world, struct RpHandle handle);

/**
 * 设置碰撞体的世界坐标位置（实际上是移动其父刚体）。
 * 因为碰撞体附着在刚体上，移动刚体就等于移动碰撞体。
 */
void rp_collider_set_position(struct RpWorld *world, struct RpHandle handle, float x, float y);

/**
 * 设置碰撞体的世界空间旋转角度（弧度）。实际上是旋转其父刚体。
 */
void rp_collider_set_rotation(struct RpWorld *world, struct RpHandle handle, float angle);

/**
 * 同时设置碰撞体的位置和旋转角度（比分别调用 set_position + set_rotation 更高效）。
 */
void rp_collider_set_pose(struct RpWorld *world,
                          struct RpHandle handle,
                          float x,
                          float y,
                          float angle);

/**
 * 获取碰撞体的世界坐标位置。
 * 如果句柄无效，返回 (0, 0)。
 */
struct RpVec2 rp_collider_get_position(const struct RpWorld *world, struct RpHandle handle);

/**
 * 获取碰撞体的世界空间旋转角度（弧度）。
 * 如果句柄无效，返回 0。
 */
float rp_collider_get_rotation(const struct RpWorld *world, struct RpHandle handle);

/**
 * 返回碰撞体是否启用。禁用的碰撞体不参与碰撞检测和查询。
 */
bool rp_collider_is_enabled(const struct RpWorld *world, struct RpHandle handle);

/**
 * 启用或禁用碰撞体。禁用的碰撞体将被排除在查询和模拟之外。
 */
void rp_collider_set_enabled(struct RpWorld *world, struct RpHandle handle, bool enabled);

/**
 * 设置碰撞体是否为传感器模式。
 * - 传感器（Sensor）：只触发重叠事件，不产生物理碰撞响应（物体可以穿过）
 * - 实体（Solid）：产生真正的物理碰撞
 * collider_type: 0 = 实体（Solid），非0 = 传感器（Sensor）
 */
void rp_collider_set_type(struct RpWorld *world,
                          struct RpHandle handle,
                          int32_t collider_type);

/**
 * 返回碰撞体类型：0 = 实体（Solid），1 = 传感器（Sensor）。
 */
int32_t rp_collider_get_type(const struct RpWorld *world, struct RpHandle handle);

/**
 * 获取碰撞体的碰撞组成员位掩码（32位）。
 * 碰撞组用于控制哪些碰撞体之间可以交互。
 */
uint32_t rp_collider_get_group(const struct RpWorld *world, struct RpHandle handle);

/**
 * 设置碰撞体的碰撞组成员位掩码（32位）。
 */
void rp_collider_set_group(struct RpWorld *world, struct RpHandle handle, uint32_t group);

/**
 * 检查句柄是否指向一个有效的碰撞体。
 * 如果碰撞体已被删除或句柄无效，返回 false。
 */
bool rp_handle_is_valid(const struct RpWorld *world, struct RpHandle handle);

/**
 * 设置刚体类型。0 = Fixed, 1 = Dynamic, 2 = KinematicPositionBased。
 */
void rp_body_set_type(struct RpWorld *world, struct RpHandle handle, int32_t body_type);

/**
 * 获取刚体类型。0 = Fixed, 1 = Dynamic, 2 = Kinematic。
 */
int32_t rp_body_get_type(const struct RpWorld *world, struct RpHandle handle);

/**
 * 获取刚体的线速度。
 */
struct RpVec2 rp_body_get_velocity(const struct RpWorld *world, struct RpHandle handle);

/**
 * 设置刚体的线速度（仅对 Dynamic 有效）。
 */
void rp_body_set_velocity(struct RpWorld *world, struct RpHandle handle, float vx, float vy);

/**
 * 获取刚体的角速度（弧度/秒）。
 */
float rp_body_get_angular_velocity(const struct RpWorld *world, struct RpHandle handle);

/**
 * 设置刚体的角速度（弧度/秒，仅对 Dynamic 有效）。
 */
void rp_body_set_angular_velocity(struct RpWorld *world, struct RpHandle handle, float angvel);

/**
 * 对刚体施加力（持续力，在下一次 step 中生效，仅对 Dynamic 有效）。
 * 力会在 step 结束后自动清零。
 */
void rp_body_apply_force(struct RpWorld *world, struct RpHandle handle, float fx, float fy);

/**
 * 对刚体施加冲量（瞬间推力，立即改变速度，仅对 Dynamic 有效）。
 */
void rp_body_apply_impulse(struct RpWorld *world, struct RpHandle handle, float ix, float iy);

/**
 * 对刚体施加扭矩冲量（瞬间改变角速度，仅对 Dynamic 有效）。
 */
void rp_body_apply_torque_impulse(struct RpWorld *world, struct RpHandle handle, float torque);

/**
 * 设置刚体的线性阻尼（用于模拟空气阻力，使物体逐渐减速）。
 * 0.0 = 无阻尼，值越大减速越快。
 */
void rp_body_set_linear_damping(struct RpWorld *world, struct RpHandle handle, float damping);

/**
 * 设置刚体的角度阻尼（使旋转逐渐减速）。
 */
void rp_body_set_angular_damping(struct RpWorld *world, struct RpHandle handle, float damping);

/**
 * 设置刚体的重力缩放系数。
 * 1.0 = 正常重力，0.0 = 无重力，2.0 = 双倍重力，-1.0 = 反重力。
 */
void rp_body_set_gravity_scale(struct RpWorld *world, struct RpHandle handle, float scale);

/**
 * 锁定刚体的旋转（防止碰撞导致物体旋转）。
 */
void rp_body_set_rotation_locked(struct RpWorld *world, struct RpHandle handle, bool locked);

/**
 * 发射一条射线，返回第一个被击中的碰撞体。
 * - `origin_x/y`: 射线起点
 * - `dir_x/y`: 射线方向（不需要是单位向量）
 * - `max_toi`: 最大检测距离（沿射线方向）
 * - `group`: 过滤位掩码——只有碰撞组成员与此掩码有交集的碰撞体才会被检测
 */
struct RpRayHit rp_query_ray_cast(const struct RpWorld *world,
                                  float origin_x,
                                  float origin_y,
                                  float dir_x,
                                  float dir_y,
                                  float max_toi,
                                  uint32_t group);

/**
 * 发射一条射线，返回第一个被击中的碰撞体，同时包含表面法线和撞击点信息。
 * 比 rp_query_ray_cast 提供更多信息，但开销稍大。
 */
struct RpRayHitWithNormal rp_query_ray_cast_with_normal(const struct RpWorld *world,
                                                        float origin_x,
                                                        float origin_y,
                                                        float dir_x,
                                                        float dir_y,
                                                        float max_toi,
                                                        uint32_t group);

/**
 * 发射一条射线，返回所有被穿过的碰撞体（不仅仅是第一个）。
 * - `out_handles`: C 端提供的输出缓冲区，用于写入命中的碰撞体句柄
 * - `max_results`: 缓冲区最大容量
 * - 返回值: 实际写入的结果数量
 */
uint32_t rp_query_ray_cast_all(const struct RpWorld *world,
                               float origin_x,
                               float origin_y,
                               float dir_x,
                               float dir_y,
                               float max_toi,
                               uint32_t group,
                               struct RpHandle *out_handles,
                               uint32_t max_results);

/**
 * 沿指定方向投射一个圆形（球形），返回第一个被碰到的碰撞体。
 * 这相当于"粗射线检测"（thick raycast）/ 扫掠测试（sweep test）。
 * 可以理解为：一个圆形沿着路径滑动，看它最先碰到什么。
 * - `radius`: 投射的圆形半径
 * - 其他参数与射线检测类似
 */
struct RpRayHit rp_query_shape_cast_circle(const struct RpWorld *world,
                                           float origin_x,
                                           float origin_y,
                                           float dir_x,
                                           float dir_y,
                                           float max_toi,
                                           float radius,
                                           uint32_t group);

/**
 * 将一个圆形沿指定方向投射，返回第一个碰到的碰撞体，同时包含碰撞点和表面法线。
 * 比 rp_query_shape_cast_circle 提供更多信息（法线和碰撞点），适用于需要反弹方向等场景。
 */
struct RpRayHitWithNormal rp_query_shape_cast_circle_with_normal(const struct RpWorld *world,
                                                                 float origin_x,
                                                                 float origin_y,
                                                                 float dir_x,
                                                                 float dir_y,
                                                                 float max_toi,
                                                                 float radius,
                                                                 uint32_t group);

/**
 * 测试两个指定的碰撞体当前是否相交（重叠）。
 * 返回 true 表示它们有重叠区域。
 */
bool rp_query_intersection_test(const struct RpWorld *world,
                                struct RpHandle handle_a,
                                struct RpHandle handle_b);

/**
 * 测试一个点是否在任何碰撞体内部。返回第一个找到的碰撞体。
 * 如果没有碰撞体包含该点，返回无效句柄。
 * 常用于：鼠标点击拾取、区域检测等。
 */
struct RpHandle rp_query_point_intersect(const struct RpWorld *world,
                                         float x,
                                         float y,
                                         uint32_t group);

/**
 * 查找所有与指定圆形区域重叠的碰撞体。
 * - `x`, `y`: 圆心位置
 * - `radius`: 圆的半径
 * - `out_handles`: 输出缓冲区
 * - `max_results`: 缓冲区容量
 * - 返回值: 实际写入的结果数量
 */
uint32_t rp_query_intersect_circle(const struct RpWorld *world,
                                   float x,
                                   float y,
                                   float radius,
                                   uint32_t group,
                                   struct RpHandle *out_handles,
                                   uint32_t max_results);

/**
 * 查找所有与指定矩形区域重叠的碰撞体。
 * - `half_width`, `half_height`: 矩形的半宽和半高
 * - `angle`: 矩形的旋转角度（弧度）
 */
uint32_t rp_query_intersect_rect(const struct RpWorld *world,
                                 float x,
                                 float y,
                                 float angle,
                                 float half_width,
                                 float half_height,
                                 uint32_t group,
                                 struct RpHandle *out_handles,
                                 uint32_t max_results);

/**
 * 查找所有与指定胶囊体区域重叠的碰撞体。
 * - `half_height`: 胶囊中间矩形部分的半高
 * - `radius`: 胶囊两端半圆的半径
 */
uint32_t rp_query_intersect_capsule(const struct RpWorld *world,
                                    float x,
                                    float y,
                                    float angle,
                                    float half_height,
                                    float radius,
                                    uint32_t group,
                                    struct RpHandle *out_handles,
                                    uint32_t max_results);

/**
 * 返回上一次 step 后的接触对数量。
 */
uint32_t rp_world_contact_pair_count(const struct RpWorld *world);

/**
 * 将接触对信息复制到 `out_pairs` 缓冲区（最多 `max_pairs` 个）。
 * 返回实际写入的数量。
 * C 端需要预先分配足够大的缓冲区（可以先调用 rp_world_contact_pair_count 获取数量）。
 */
uint32_t rp_world_get_contact_pairs(const struct RpWorld *world,
                                    struct RpContactPairInfo *out_pairs,
                                    uint32_t max_pairs);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* RAPIER2D_FFI_H */
