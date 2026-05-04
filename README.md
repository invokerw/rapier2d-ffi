# rapier2d-ffi

[rapier2d](https://rapier.rs/) 物理引擎的 C 绑定（f32 精度，增强确定性）。

编译产出静态库 (`librapier2d_ffi.a`) 和动态库 (`librapier2d_ffi.dylib` / `.so`)，附带自动生成的 C 头文件 `rapier2d_ffi.h`。

## 环境要求

- Rust >= 1.86（edition 2024）
- C 编译器（cc / clang / gcc）

## 编译

### 当前平台

```bash
cargo build --release
```

产出文件：

| 文件 | 路径 |
|------|------|
| 静态库 | `target/release/librapier2d_ffi.a` |
| 动态库 | `target/release/librapier2d_ffi.dylib`（macOS）/ `.so`（Linux） |
| C 头文件 | `rapier2d_ffi.h`（项目根目录，cbindgen 自动生成） |

### 跨平台编译

项目提供了 `build.sh` 脚本，可以一键编译 macOS / Linux / Windows 的库文件。

```bash
# 首次使用：安装所需的 Rust target 并查看工具链要求
./build.sh setup

# 编译所有平台
./build.sh all

# 或只编译某个平台
./build.sh mac       # macOS (arm64 + x86_64 Universal Binary)
./build.sh linux     # Linux x86_64
./build.sh windows   # Windows x86_64
```

交叉编译的前置依赖（在 macOS 上）：

```bash
# Linux 交叉编译器
brew tap messense/macos-cross-toolchains
brew install x86_64-unknown-linux-gnu

# Windows 交叉编译器
brew install mingw-w64
```

编译产物输出到 `dist/` 目录：

```text
dist/
├── mac/
│   ├── librapier2d_ffi.dylib   # macOS 动态库 (Universal Binary)
│   ├── librapier2d_ffi.a       # macOS 静态库 (Universal Binary)
│   └── rapier2d_ffi.h
├── linux-x86_64/
│   ├── librapier2d_ffi.so      # Linux 动态库
│   ├── librapier2d_ffi.a       # Linux 静态库
│   └── rapier2d_ffi.h
└── windows-x86_64/
    ├── rapier2d_ffi.dll         # Windows 动态库
    ├── librapier2d_ffi.a        # Windows 静态库
    └── rapier2d_ffi.h
```

## 运行测试

### 单元测试

```bash
# macOS
cc -o test/test_rapier2d test/test_rapier2d.c \
   -I. -Ltarget/release -lrapier2d_ffi \
   -framework Security -framework CoreFoundation -lm
DYLD_LIBRARY_PATH=target/release ./test/test_rapier2d

# Linux
cc -o test/test_rapier2d test/test_rapier2d.c \
   -I. -Ltarget/release -lrapier2d_ffi -lm -lpthread -ldl
LD_LIBRARY_PATH=target/release ./test/test_rapier2d
```

测试包含 31 个场景、196 个断言，覆盖所有 API 功能、确定性验证和 NULL 安全。

### 可视化测试（raylib）

基于 [raylib](https://www.raylib.com/) 的图形化测试，支持 macOS / Linux / Windows。

```bash
# 安装 raylib（首次）
./test/build_visual_test.sh setup

# 编译并运行
./test/build_visual_test.sh

# 或分步操作
./test/build_visual_test.sh build
./test/build_visual_test.sh run
```

可视化测试包含：重力模拟、动态碰撞、射线检测、区域查询、传感器区域、拖拽交互等演示。

操作方式：

| 按键 | 功能 |
|------|------|
| 鼠标左键 | 射线检测 |
| 鼠标右键 | 区域查询 |
| D + 左键拖拽 | 拖动物体（切换为 Kinematic） |
| Space | 生成新球体 |
| F | 对所有动态体施加向上冲量 |
| G | 切换重力开关 |
| R | 重置场景 |

## 快速上手

```c
#include "rapier2d_ffi.h"

void on_collision_start(RpHandle a, RpHandle b) { /* ... */ }

int main(void) {
    // 创建世界（重力、碰撞回调）
    RpWorld *world = rp_world_create(0.0f, -9.81f, on_collision_start, NULL);

    // 添加动态球体（x, y, 半径, collider_type, body_type, 碰撞组）
    //   collider_type: 0 = 实体, 1 = 传感器
    //   body_type:     0 = 固定, 1 = 动态, 2 = 运动学
    RpHandle ball   = rp_collider_create_circle(world, 0, 5, 1.0f, 0, 1, 0xFFFFFFFF);
    // 添加固定地面
    RpHandle ground = rp_collider_create_rect(world, 0, 0, 0, 50, 0.5f, 0, 0, 0xFFFFFFFF);

    // 步进模拟（动态球体会受重力下落）
    for (int i = 0; i < 60; i++) {
        rp_world_step(world);
    }

    // 读取位置
    RpVec2 pos = rp_collider_get_position(world, ball);

    // 对动态体施加冲量
    rp_body_apply_impulse(world, ball, 10.0f, 0.0f);

    // 射线检测
    RpRayHit hit = rp_query_ray_cast(world, 0, 10, 0, -1, 100, 0xFFFFFFFF);

    // 清理
    rp_collider_destroy(world, ball);
    rp_collider_destroy(world, ground);
    rp_world_destroy(world);
    return 0;
}
```

## API 概览

### 数据类型

| 类型 | 说明 |
|------|------|
| `RpWorld *` | 不透明的物理世界指针，通过 API 创建/销毁 |
| `RpHandle` | 碰撞体句柄 `{id, generation}`，用 `rp_handle_is_valid()` 检查有效性 |
| `RpVec2` | 二维向量 `{x, y}` |
| `RpRayHit` | 射线检测结果 `{handle, toi, hit}` |
| `RpRayHitWithNormal` | 射线检测结果，附带命中点和表面法线 |
| `RpContactPairInfo` | 接触对信息 `{collider1, collider2, has_any_active_contact}` |

### 回调类型

| 类型 | 签名 |
|------|------|
| `RpLogCallback` | `void (*)(const char *)` |
| `RpCollisionStartCallback` | `void (*)(RpHandle, RpHandle)` |
| `RpCollisionStopCallback` | `void (*)(RpHandle, RpHandle)` |

传 `NULL` 可禁用对应回调。

### 世界管理

```c
RpWorld *rp_world_create(gravity_x, gravity_y, start_cb, stop_cb);  // 创建世界
void     rp_world_destroy(RpWorld *world);                           // 销毁世界
void     rp_world_step(RpWorld *world);                              // 步进一帧
void     rp_world_update_query_pipeline(RpWorld *world);             // 刷新查询用的 BVH（见下文）
void     rp_world_set_timestep(RpWorld *world, float dt);            // 设置时间步长，默认 1/60
void     rp_world_set_gravity(RpWorld *world, float x, float y);    // 设置重力
uint32_t rp_world_collider_count(const RpWorld *world);              // 碰撞体数量
```

**关于 `rp_world_update_query_pipeline`：** 新创建的碰撞体，以及通过 `rp_collider_set_position / set_rotation / set_pose` 移动过的碰撞体，在下一次 `rp_world_step` 之前都不会出现在宽相 BVH 中——射线检测、形状投射、点/区域查询都找不到它们。如果需要在不推进物理模拟的前提下立即查询刚创建或移动后的碰撞体，就在查询前调用一次此函数：

```c
rp_collider_create_circle(world, 5, 0, 1, 0, 0, 0xFFFFFFFF);
rp_world_update_query_pipeline(world);  // 把新 collider 塞进 BVH
rp_query_ray_cast(world, 0, 0, 1, 0, 10, 0xFFFFFFFF);  // 现在能命中
```

此函数只刷新查询索引，不会推进模拟、不会触发碰撞回调。`rp_world_step` 会自动完成同样的刷新，因此正常的「每帧 step」循环里不需要调它。

### 碰撞体创建

每个碰撞体内部会挂载到一个独立的刚体上。公共参数：
- `collider_type`：`0` = 实体，非零 = 传感器（仅触发事件，无物理响应）
- `body_type`：`0` = 固定（Fixed），`1` = 动态（Dynamic），`2` = 运动学（KinematicPositionBased）
- `group`：32 位碰撞组成员掩码

```c
RpHandle rp_collider_create_circle(world, x, y, radius, type, body_type, group);                    // 圆形
RpHandle rp_collider_create_rect(world, x, y, angle, half_w, half_h, type, body_type, group);       // 矩形
RpHandle rp_collider_create_triangle(world, x, y, angle, a, b, c, type, body_type, group);          // 三角形
RpHandle rp_collider_create_convex(world, x, y, angle, vertices, count, type, body_type, group);    // 凸多边形
RpHandle rp_collider_create_capsule(world, x, y, angle, half_height, radius, type, body_type, group); // 胶囊
RpHandle rp_collider_create_segment(world, x, y, angle, a, b, type, body_type, group);              // 线段
bool     rp_collider_destroy(world, handle);                                                          // 销毁
```

### 刚体类型

| 值 | 类型 | 说明 |
|----|------|------|
| `0` | Fixed（固定） | 不受力和重力影响，完全静止。适用于地面、墙壁等 |
| `1` | Dynamic（动态） | 受重力、力、冲量影响，参与完整的物理模拟 |
| `2` | KinematicPositionBased（运动学） | 由代码控制位置，不受力影响，但会推开动态体 |

### 碰撞体属性

```c
// 位置与旋转
void     rp_collider_set_position(world, handle, x, y);
void     rp_collider_set_rotation(world, handle, angle);     // 弧度
void     rp_collider_set_pose(world, handle, x, y, angle);   // 同时设置位置和旋转
RpVec2   rp_collider_get_position(world, handle);
float    rp_collider_get_rotation(world, handle);

// 启用/禁用
bool     rp_collider_is_enabled(world, handle);
void     rp_collider_set_enabled(world, handle, enabled);

// 实体/传感器
int32_t  rp_collider_get_type(world, handle);                // 0=实体, 1=传感器
void     rp_collider_set_type(world, handle, type);

// 碰撞组
uint32_t rp_collider_get_group(world, handle);
void     rp_collider_set_group(world, handle, group);

// 有效性检查
bool     rp_handle_is_valid(world, handle);
```

### 刚体控制（Dynamic / Kinematic）

```c
// 刚体类型切换
void    rp_body_set_type(world, handle, body_type);           // 0=Fixed, 1=Dynamic, 2=Kinematic
int32_t rp_body_get_type(world, handle);

// 线速度
RpVec2  rp_body_get_velocity(world, handle);
void    rp_body_set_velocity(world, handle, vx, vy);

// 角速度
float   rp_body_get_angular_velocity(world, handle);
void    rp_body_set_angular_velocity(world, handle, omega);

// 力与冲量（仅对 Dynamic 有效）
void    rp_body_apply_force(world, handle, fx, fy);           // 持续力（下一步生效）
void    rp_body_apply_impulse(world, handle, ix, iy);         // 瞬时冲量
void    rp_body_apply_torque_impulse(world, handle, torque);  // 角冲量

// 阻尼
void    rp_body_set_linear_damping(world, handle, damping);   // 线性阻尼
void    rp_body_set_angular_damping(world, handle, damping);  // 角阻尼

// 重力与旋转
void    rp_body_set_gravity_scale(world, handle, scale);      // 重力缩放（0=无重力, 1=正常, -1=反重力）
void    rp_body_set_rotation_locked(world, handle, locked);   // 锁定旋转
```

### 查询

```c
// 射线检测
RpRayHit           rp_query_ray_cast(world, ox, oy, dx, dy, max_toi, group);            // 最近命中
RpRayHitWithNormal rp_query_ray_cast_with_normal(world, ox, oy, dx, dy, max_toi, group); // 附带法线
uint32_t           rp_query_ray_cast_all(world, ox, oy, dx, dy, max_toi, group, out, max); // 穿透所有

// 形状投射（圆形扫掠，类似"胖射线"；已穿透时不会在 toi=0 处阻塞，允许逃离重叠）
// `exclude_handle` 用于把某个 collider 从结果中排除（例如排除投射者自身）；
// 不需要排除时传 `{0xFFFFFFFF, 0xFFFFFFFF}`（即 RpHandle::invalid()）。
RpRayHit           rp_query_shape_cast_circle(world, ox, oy, dx, dy, max_toi, radius, group, exclude_handle);             // 最近命中
RpRayHitWithNormal rp_query_shape_cast_circle_with_normal(world, ox, oy, dx, dy, max_toi, radius, group, exclude_handle); // 附带法线

// 相交测试
bool     rp_query_intersection_test(world, handle_a, handle_b);  // 两个碰撞体是否相交
RpHandle rp_query_point_intersect(world, x, y, group);           // 点查询

// 区域查询（结果写入调用者提供的缓冲区，返回写入数量）
uint32_t rp_query_intersect_circle(world, x, y, radius, group, out, max);              // 圆形区域
uint32_t rp_query_intersect_rect(world, x, y, angle, half_w, half_h, group, out, max); // 矩形区域
uint32_t rp_query_intersect_capsule(world, x, y, angle, half_h, radius, group, out, max); // 胶囊区域
uint32_t rp_query_intersect_sector(world, x, y, radius, start_angle, sweep_angle, segments, group, out, max); // 扇形区域

// 寻找空旷点
RpVec2   rp_query_find_clear_point(world, x, y, r, r2, ignore_types, group, mode, max_attempts, &found);          // 圆形搜索区域
RpVec2   rp_query_find_clear_point_in_rect(world, rect_x, rect_y, rect_w, rect_h, r2, ignore_types, group, mode, max_attempts, &found); // 矩形搜索区域

// 接触对
uint32_t rp_world_contact_pair_count(world);                     // 接触对数量
uint32_t rp_world_get_contact_pairs(world, out_pairs, max_pairs); // 获取接触对详情
```

### 日志

```c
void rp_register_log_callback(RpLogCallback cb);   // 传 NULL 禁用日志
```

### 寻找空旷点

在搜索区域内寻找一个"空旷"的点，该点周围 `r2` 半径内没有指定类型的碰撞体。使用确定性伪随机采样，相同参数多次调用返回相同结果。提供两个变体：

**圆形搜索区域：**

```c
bool found;
RpVec2 point = rp_query_find_clear_point(
    world,
    0.0f, 0.0f,     // 搜索圆心
    10.0f,           // 搜索半径 r
    2.0f,            // 清空半径 r2（候选点周围不能有碰撞体的范围）
    0x1,             // ignore_collider_types（见下表）
    0xFFFFFFFF,      // 碰撞组过滤
    0,               // mode: 0=均匀随机, 1=优先靠近中心
    100,             // 最大尝试次数
    &found           // 输出：是否找到
);
if (found) {
    // 使用 point.x, point.y
}
```

**矩形搜索区域：**

```c
bool found;
RpVec2 point = rp_query_find_clear_point_in_rect(
    world,
    -10.0f, -5.0f,  // 搜索矩形左下角坐标 (rect_x, rect_y)
    20.0f, 10.0f,   // 搜索矩形宽度和高度 (rect_w, rect_h)
    2.0f,            // 清空半径 r2
    0x1,             // ignore_collider_types（见下表）
    0xFFFFFFFF,      // 碰撞组过滤
    0,               // mode: 0=均匀随机, 1=优先靠近中心
    100,             // 最大尝试次数
    &found           // 输出：是否找到
);
if (found) {
    // 使用 point.x, point.y
}
```

**`ignore_collider_types` 位掩码：**

| 值 | 含义 |
|----|------|
| `0x1` | 避开实体碰撞体（Solid, collider_type == 0） |
| `0x2` | 避开传感器碰撞体（Sensor, collider_type != 0） |
| `0x3` | 避开所有碰撞体 |

**`mode` 采样模式（两个变体均支持）：**

| 值 | 策略 | 适用场景 |
|----|------|---------|
| `0` | 均匀随机采样（搜索区域内各处概率相同） | 随机刷怪、随机放置物品 |
| `1` | 优先靠近中心（先尝试中心点，逐渐向外扩展） | 围绕目标点生成、就近寻找空位 |

## 扇形区域查询

`rp_query_intersect_sector` 以 `(x, y)` 为顶点，从 `start_angle` 方向**逆时针**扫过 `sweep_angle` 弧度，外圆半径为 `radius`，返回所有与此扇形重叠的碰撞体。

```c
RpHandle hits[16];
// 以原点为顶点，朝向 +X 轴，±45° 共 90° 扇形，半径 10
uint32_t n = rp_query_intersect_sector(
    world,
    0.0f, 0.0f,        // 扇形顶点
    10.0f,             // 外圆半径
    -0.785398f,        // start_angle：-π/4（从 -45° 方向起）
    1.570796f,         // sweep_angle：π/2（共扫过 90°）
    12,                // segments：弧段细分，建议 8~16
    0xFFFFFFFF,        // 碰撞组过滤
    hits, 16
);
```

参数说明：
- `start_angle`：起始方向的弧度角（0 = +X 轴，π/2 = +Y 轴）。
- `sweep_angle`：扫过的弧度，必须 > 0。
  - `>= 2π` 时退化为整圆，等价于 `rp_query_intersect_circle`。
  - `∈ (π, 2π)` 时内部拆成两个凸子扇形通过 `Compound` 合并（单个凸多边形无法表达 > 180°）。
  - `<= π` 时为单个凸多边形，最常用。
- `segments`：弧段细分数量，最少 2。越大越贴近真实圆弧，开销越高。典型值 8~16 即可满足视觉需求。
- 非法参数（`radius <= 0`、`sweep_angle <= 0`、`world == NULL`）返回 0。

相比先用 `intersect_circle` 粗筛、再按角度过滤的朴素方案，此接口基于 parry 的凸形相交测试，能正确命中"中心在扇外、但边缘伸进扇内"的大体积碰撞体。

## 碰撞组

碰撞组使用 32 位掩码。每个碰撞体有一个 **membership**（属于哪些组），查询时传入 **filter**（测试哪些组）。当 `(membership & filter) != 0` 时碰撞体会被包含在结果中。

```c
#define GROUP_PLAYER  (1 << 0)   // 0x01
#define GROUP_ENEMY   (1 << 1)   // 0x02
#define GROUP_TERRAIN (1 << 2)   // 0x04

// 动态玩家属于 GROUP_PLAYER
rp_collider_create_circle(world, 0, 0, 1, 0, 1, GROUP_PLAYER);

// 固定地形
rp_collider_create_rect(world, 0, -5, 0, 50, 0.5f, 0, 0, GROUP_TERRAIN);

// 射线只检测地形
RpRayHit hit = rp_query_ray_cast(world, 0, 10, 0, -1, 100, GROUP_TERRAIN);

// 射线检测玩家和地形
RpRayHit hit2 = rp_query_ray_cast(world, 0, 10, 0, -1, 100, GROUP_PLAYER | GROUP_TERRAIN);
```

## NULL 安全

所有函数均可安全接收 `NULL` 世界指针（返回零值/false/空结果，不会崩溃）。无效句柄返回默认值。

## 项目结构

```
rapier2d-ffi/
├── Cargo.toml                    # Rust 项目配置
├── build.rs                      # cbindgen 头文件生成
├── build.sh                      # 跨平台编译脚本（macOS/Linux/Windows）
├── cbindgen.toml                 # cbindgen 配置
├── src/
│   └── lib.rs                    # FFI 实现
├── rapier2d_ffi.h                # 自动生成的 C 头文件（勿手动编辑）
├── CLAUDE.md                     # Claude Code 工作指南
├── test/
│   ├── test_rapier2d.c           # C 单元测试（31 个场景，196 个断言）
│   ├── visual_test.c             # raylib 可视化测试
│   └── build_visual_test.sh      # 可视化测试编译脚本
└── README.md
```

## 依赖

| 库 | 版本 | 说明 |
|----|------|------|
| [rapier2d](https://github.com/dimforge/rapier) | 0.32.0 | 启用 `enhanced-determinism` 特性 |
| [parry2d](https://github.com/dimforge/parry) | 0.26.0 | 启用 `enhanced-determinism` 特性 |
| [cbindgen](https://github.com/mozilla/cbindgen) | 0.28 | 编译期自动生成头文件 |

`Cargo.toml` 中的 `[patch.crates-io]` 将 rapier2d 指向 git master 分支，可根据需要改为你自己的 fork 或使用 crates.io 发布版。

## 许可证

MIT License
