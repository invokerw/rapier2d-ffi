/**
 * rapier2d-ffi 可视化测试 (基于 raylib)
 *
 * 功能:
 *   - 展示各种碰撞体形状（圆、矩形、三角形、胶囊、线段）
 *   - 动态物体受重力影响会自由落体，碰到地面和其他物体会弹开
 *   - 鼠标左键: 从屏幕顶部向鼠标位置发射射线（显示命中点和法线）
 *   - 鼠标右键: 在鼠标位置做圆形区域查询（高亮重叠的碰撞体）
 *   - 键盘 D:   拖拽模式 - 按住 D 再左键点击拖拽碰撞体
 *   - 空格键:   在鼠标位置生成一个新的动态圆形
 *   - 键盘 F:   对所有动态物体施加向上的冲量（弹飞）
 *   - 键盘 R:   重置场景
 *   - 碰撞事件会实时高亮显示
 *
 * 编译: 见 test/build_visual_test.sh
 */

#include <stdio.h>
#include <math.h>
#include "raylib.h"
#include "rapier2d_ffi.h"

/* ======================================================================
 * 常量
 * ====================================================================== */

#define SCREEN_W    1024
#define SCREEN_H    768
#define MAX_BODIES  128
#define PIXELS_PER_METER 50.0f  /* 1 物理单位 = 50 像素 */
#define ALL_GROUPS  0xFFFFFFFF

/* 刚体类型常量 (与 lib.rs 中的 body_type_from_i32 对应) */
#define BODY_FIXED    0
#define BODY_DYNAMIC  1
#define BODY_KINEMATIC 2

/* ======================================================================
 * 坐标转换
 *
 * 物理世界: Y 轴向上, 原点在屏幕中下方
 * 屏幕:     Y 轴向下, 原点在左上角
 * ====================================================================== */

static inline Vector2 world_to_screen(float wx, float wy) {
    return (Vector2){
        SCREEN_W / 2.0f + wx * PIXELS_PER_METER,
        SCREEN_H * 0.75f - wy * PIXELS_PER_METER
    };
}

static inline void screen_to_world(Vector2 sp, float *wx, float *wy) {
    *wx = (sp.x - SCREEN_W / 2.0f) / PIXELS_PER_METER;
    *wy = (SCREEN_H * 0.75f - sp.y) / PIXELS_PER_METER;
}

/* ======================================================================
 * 碰撞体信息（用于渲染）
 * ====================================================================== */

typedef enum {
    SHAPE_CIRCLE,
    SHAPE_RECT,
    SHAPE_TRIANGLE,
    SHAPE_CAPSULE,
    SHAPE_SEGMENT,
} ShapeKind;

typedef struct {
    RpHandle handle;
    ShapeKind kind;
    Color color;
    bool highlight;     /* 碰撞高亮 */
    bool query_hit;     /* 查询高亮 */
    int body_type;      /* 0=Fixed, 1=Dynamic, 2=Kinematic */
    /* 形状参数 (创建时记录, 用于渲染) */
    float radius;
    float half_w, half_h;
    RpVec2 a, b, c;     /* 三角形/线段顶点 */
} BodyInfo;

static BodyInfo g_bodies[MAX_BODIES];
static int g_body_count = 0;

/* ======================================================================
 * 碰撞事件状态
 * ====================================================================== */

typedef struct { RpHandle a, b; float timer; } CollisionFlash;
#define MAX_FLASHES 32
static CollisionFlash g_flashes[MAX_FLASHES];
static int g_flash_count = 0;

static bool handle_eq(RpHandle a, RpHandle b) {
    return a.id == b.id && a.generation == b.generation;
}

static void on_collision_start(RpHandle a, RpHandle b) {
    if (g_flash_count < MAX_FLASHES) {
        g_flashes[g_flash_count++] = (CollisionFlash){ a, b, 0.5f };
    }
    for (int i = 0; i < g_body_count; i++) {
        if (handle_eq(g_bodies[i].handle, a) || handle_eq(g_bodies[i].handle, b)) {
            g_bodies[i].highlight = true;
        }
    }
}

static void on_collision_stop(RpHandle a, RpHandle b) {
    for (int i = 0; i < g_body_count; i++) {
        if (handle_eq(g_bodies[i].handle, a) || handle_eq(g_bodies[i].handle, b)) {
            g_bodies[i].highlight = false;
        }
    }
}

/* ======================================================================
 * 辅助函数
 * ====================================================================== */

static int find_body_index(RpHandle h) {
    for (int i = 0; i < g_body_count; i++) {
        if (handle_eq(g_bodies[i].handle, h)) return i;
    }
    return -1;
}

static void add_body(RpHandle h, ShapeKind kind, Color col, int btype,
                     float radius, float hw, float hh,
                     RpVec2 a, RpVec2 b, RpVec2 c) {
    if (g_body_count >= MAX_BODIES) return;
    g_bodies[g_body_count++] = (BodyInfo){
        .handle = h, .kind = kind, .color = col, .body_type = btype,
        .radius = radius, .half_w = hw, .half_h = hh,
        .a = a, .b = b, .c = c,
    };
}

/* ======================================================================
 * 场景创建
 * ====================================================================== */

static RpWorld *g_world = NULL;

static void create_scene(void) {
    if (g_world) rp_world_destroy(g_world);
    g_body_count = 0;
    g_flash_count = 0;

    /* 创建带重力的世界 (Y轴向下 -9.81) */
    g_world = rp_world_create(0.0f, -9.81f, on_collision_start, on_collision_stop);

    RpVec2 zero = {0, 0};

    /* ---- 静态物体 (Fixed) ---- */

    /* 地面 */
    {
        RpHandle h = rp_collider_create_rect(g_world, 0, -3.0f, 0,
                                             9.0f, 0.3f, 0, BODY_FIXED, ALL_GROUPS);
        add_body(h, SHAPE_RECT, DARKGRAY, BODY_FIXED, 0, 9.0f, 0.3f, zero, zero, zero);
    }

    /* 左墙 */
    {
        RpHandle h = rp_collider_create_rect(g_world, -9.0f, 2.0f, 0,
                                             0.3f, 6.0f, 0, BODY_FIXED, ALL_GROUPS);
        add_body(h, SHAPE_RECT, DARKGRAY, BODY_FIXED, 0, 0.3f, 6.0f, zero, zero, zero);
    }

    /* 右墙 */
    {
        RpHandle h = rp_collider_create_rect(g_world, 9.0f, 2.0f, 0,
                                             0.3f, 6.0f, 0, BODY_FIXED, ALL_GROUPS);
        add_body(h, SHAPE_RECT, DARKGRAY, BODY_FIXED, 0, 0.3f, 6.0f, zero, zero, zero);
    }

    /* 斜坡 (倾斜的矩形) */
    {
        RpHandle h = rp_collider_create_rect(g_world, -3.5f, 0.5f, -0.35f,
                                             3.0f, 0.15f, 0, BODY_FIXED, ALL_GROUPS);
        add_body(h, SHAPE_RECT, (Color){100, 100, 100, 255}, BODY_FIXED,
                 0, 3.0f, 0.15f, zero, zero, zero);
    }

    /* 另一个斜坡 (反向) */
    {
        RpHandle h = rp_collider_create_rect(g_world, 3.5f, 2.5f, 0.35f,
                                             3.0f, 0.15f, 0, BODY_FIXED, ALL_GROUPS);
        add_body(h, SHAPE_RECT, (Color){100, 100, 100, 255}, BODY_FIXED,
                 0, 3.0f, 0.15f, zero, zero, zero);
    }

    /* ---- 动态物体 (Dynamic) - 会受重力影响 ---- */

    /* 动态圆形 */
    {
        RpHandle h = rp_collider_create_circle(g_world, -4.0f, 5.0f,
                                               0.5f, 0, BODY_DYNAMIC, ALL_GROUPS);
        add_body(h, SHAPE_CIRCLE, (Color){70, 130, 180, 255}, BODY_DYNAMIC,
                 0.5f, 0, 0, zero, zero, zero);
    }

    /* 动态矩形 */
    {
        RpHandle h = rp_collider_create_rect(g_world, -1.0f, 6.0f, 0.3f,
                                             0.6f, 0.4f, 0, BODY_DYNAMIC, ALL_GROUPS);
        add_body(h, SHAPE_RECT, (Color){60, 179, 113, 255}, BODY_DYNAMIC,
                 0, 0.6f, 0.4f, zero, zero, zero);
    }

    /* 动态三角形 */
    {
        RpVec2 a = { 0.0f,  0.6f};
        RpVec2 b = {-0.5f, -0.3f};
        RpVec2 c = { 0.5f, -0.3f};
        RpHandle h = rp_collider_create_triangle(g_world, 2.0f, 7.0f, 0.0f,
                                                 a, b, c, 0, BODY_DYNAMIC, ALL_GROUPS);
        add_body(h, SHAPE_TRIANGLE, (Color){255, 165, 0, 255}, BODY_DYNAMIC,
                 0, 0, 0, a, b, c);
    }

    /* 动态胶囊 */
    {
        RpHandle h = rp_collider_create_capsule(g_world, 4.0f, 5.0f, 0.0f,
                                                0.5f, 0.3f, 0, BODY_DYNAMIC, ALL_GROUPS);
        add_body(h, SHAPE_CAPSULE, (Color){186, 85, 211, 255}, BODY_DYNAMIC,
                 0.3f, 0.3f, 0.5f, zero, zero, zero);
    }

    /* 第二个动态圆 (稍大) */
    {
        RpHandle h = rp_collider_create_circle(g_world, 1.0f, 8.0f,
                                               0.7f, 0, BODY_DYNAMIC, ALL_GROUPS);
        add_body(h, SHAPE_CIRCLE, (Color){255, 100, 100, 255}, BODY_DYNAMIC,
                 0.7f, 0, 0, zero, zero, zero);
    }

    /* 传感器 (固定, 检测区域) */
    {
        RpHandle h = rp_collider_create_circle(g_world, 0.0f, -1.0f,
                                               2.0f, 1, BODY_FIXED, ALL_GROUPS);
        add_body(h, SHAPE_CIRCLE, (Color){255, 255, 100, 80}, BODY_FIXED,
                 2.0f, 0, 0, zero, zero, zero);
    }
}

/* ======================================================================
 * 渲染碰撞体
 * ====================================================================== */

static void draw_body(const BodyInfo *bi, const RpWorld *world) {
    RpVec2 pos = rp_collider_get_position(world, bi->handle);
    float angle = rp_collider_get_rotation(world, bi->handle);
    Vector2 sp = world_to_screen(pos.x, pos.y);
    Color col = bi->color;

    /* 高亮效果 */
    if (bi->highlight) {
        col = (Color){255, 50, 50, 255};
    }
    if (bi->query_hit) {
        col = YELLOW;
    }

    float s = PIXELS_PER_METER;

    switch (bi->kind) {
    case SHAPE_CIRCLE: {
        float r = bi->radius * s;
        int32_t ctype = rp_collider_get_type(world, bi->handle);
        if (ctype == 1) {
            /* 传感器: 虚线风格 */
            DrawCircleLinesV(sp, r, col);
            DrawCircleV(sp, r, (Color){col.r, col.g, col.b, 40});
        } else {
            DrawCircleV(sp, r, col);
            DrawCircleLinesV(sp, r, WHITE);
            /* Dynamic 物体画一条线表示旋转方向 */
            if (bi->body_type == BODY_DYNAMIC) {
                Vector2 dir = {
                    sp.x + cosf(-angle) * r * 0.7f,
                    sp.y + sinf(-angle) * r * 0.7f
                };
                DrawLineEx(sp, dir, 2.0f, WHITE);
            }
        }
        break;
    }
    case SHAPE_RECT: {
        float w = bi->half_w * 2.0f * s;
        float h = bi->half_h * 2.0f * s;
        Rectangle rect = { sp.x, sp.y, w, h };
        Vector2 origin = { w / 2.0f, h / 2.0f };
        DrawRectanglePro(rect, origin, -angle * (180.0f / PI), col);
        break;
    }
    case SHAPE_TRIANGLE: {
        float ca = cosf(angle), sa = sinf(angle);
        RpVec2 verts[3] = { bi->a, bi->b, bi->c };
        Vector2 sv[3];
        for (int i = 0; i < 3; i++) {
            float lx = verts[i].x, ly = verts[i].y;
            float wx = pos.x + lx * ca - ly * sa;
            float wy = pos.y + lx * sa + ly * ca;
            sv[i] = world_to_screen(wx, wy);
        }
        DrawTriangle(sv[0], sv[2], sv[1], col);
        DrawTriangleLines(sv[0], sv[2], sv[1], WHITE);
        break;
    }
    case SHAPE_CAPSULE: {
        float r = bi->radius * s;
        float ca = cosf(angle), sa = sinf(angle);
        float top_wx = pos.x - bi->half_h * sa;
        float top_wy = pos.y + bi->half_h * ca;
        float bot_wx = pos.x + bi->half_h * sa;
        float bot_wy = pos.y - bi->half_h * ca;
        Vector2 top_sp = world_to_screen(top_wx, top_wy);
        Vector2 bot_sp = world_to_screen(bot_wx, bot_wy);
        DrawCircleV(top_sp, r, col);
        DrawCircleV(bot_sp, r, col);
        float w = bi->radius * 2.0f * s;
        float h = bi->half_h * 2.0f * s;
        Rectangle rect = { sp.x, sp.y, w, h };
        Vector2 origin = { w / 2.0f, h / 2.0f };
        DrawRectanglePro(rect, origin, -angle * (180.0f / PI), col);
        break;
    }
    case SHAPE_SEGMENT: {
        float ca = cosf(angle), sa = sinf(angle);
        float ax = pos.x + bi->a.x * ca - bi->a.y * sa;
        float ay = pos.y + bi->a.x * sa + bi->a.y * ca;
        float bx = pos.x + bi->b.x * ca - bi->b.y * sa;
        float by = pos.y + bi->b.x * sa + bi->b.y * ca;
        Vector2 sa2 = world_to_screen(ax, ay);
        Vector2 sb = world_to_screen(bx, by);
        DrawLineEx(sa2, sb, 3.0f, col);
        DrawCircleV(sa2, 4, col);
        DrawCircleV(sb, 4, col);
        break;
    }
    }

    /* Dynamic 物体显示速度箭头 */
    if (bi->body_type == BODY_DYNAMIC && bi->kind != SHAPE_CIRCLE) {
        RpVec2 vel = rp_body_get_velocity(world, bi->handle);
        float vlen = sqrtf(vel.x * vel.x + vel.y * vel.y);
        if (vlen > 0.5f) {
            float scale = 10.0f; /* 像素/速度 */
            Vector2 vend = {
                sp.x + vel.x * scale,
                sp.y - vel.y * scale  /* Y 轴翻转 */
            };
            DrawLineEx(sp, vend, 1.5f, (Color){100, 255, 100, 150});
        }
    }
}

/* ======================================================================
 * 射线可视化
 * ====================================================================== */

typedef struct {
    bool active;
    float ox, oy, dx, dy;
    RpRayHitWithNormal hit;
    float timer;
} RayVis;

static RayVis g_ray = {0};

static void draw_ray(void) {
    if (!g_ray.active) return;

    Vector2 start = world_to_screen(g_ray.ox, g_ray.oy);

    if (g_ray.hit.hit) {
        Vector2 hit_sp = world_to_screen(g_ray.hit.point.x, g_ray.hit.point.y);
        DrawLineEx(start, hit_sp, 2.0f, GREEN);
        DrawCircleV(hit_sp, 6.0f, RED);
        float nl = 40.0f;
        Vector2 normal_end = {
            hit_sp.x + g_ray.hit.normal.x * nl,
            hit_sp.y - g_ray.hit.normal.y * nl
        };
        DrawLineEx(hit_sp, normal_end, 2.0f, ORANGE);
    } else {
        float end_x = g_ray.ox + g_ray.dx * 100.0f;
        float end_y = g_ray.oy + g_ray.dy * 100.0f;
        Vector2 end = world_to_screen(end_x, end_y);
        DrawLineEx(start, end, 1.0f, (Color){0, 255, 0, 100});
    }
}

/* ======================================================================
 * 圆形区域查询可视化
 * ====================================================================== */

typedef struct {
    bool active;
    float x, y, radius;
    float timer;
} QueryVis;

static QueryVis g_query = {0};

static void draw_query(void) {
    if (!g_query.active) return;
    Vector2 sp = world_to_screen(g_query.x, g_query.y);
    float r = g_query.radius * PIXELS_PER_METER;
    DrawCircleLinesV(sp, r, (Color){255, 255, 0, 200});
    DrawCircleV(sp, r, (Color){255, 255, 0, 30});
}

/* ======================================================================
 * 拖拽
 * ====================================================================== */

typedef struct {
    bool active;
    int body_idx;
    float offset_x, offset_y;
} DragState;

static DragState g_drag = {0};

/* 随机颜色 */
static Color random_color(void) {
    return (Color){
        (unsigned char)(100 + GetRandomValue(0, 155)),
        (unsigned char)(100 + GetRandomValue(0, 155)),
        (unsigned char)(100 + GetRandomValue(0, 155)),
        255
    };
}

/* ======================================================================
 * 主程序
 * ====================================================================== */

int main(void) {
    InitWindow(SCREEN_W, SCREEN_H, "rapier2d-ffi visual test");
    SetTargetFPS(60);

    create_scene();

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        Vector2 mouse = GetMousePosition();
        float mx, my;
        screen_to_world(mouse, &mx, &my);

        /* ---------- 键盘: R 重置 ---------- */
        if (IsKeyPressed(KEY_R)) {
            create_scene();
            g_ray.active = false;
            g_query.active = false;
            g_drag.active = false;
        }

        /* ---------- 空格: 在鼠标位置生成动态圆 ---------- */
        if (IsKeyPressed(KEY_SPACE) && g_body_count < MAX_BODIES) {
            float r = 0.3f + (float)GetRandomValue(0, 40) / 100.0f;
            RpVec2 zero = {0, 0};
            RpHandle h = rp_collider_create_circle(g_world, mx, my,
                                                   r, 0, BODY_DYNAMIC, ALL_GROUPS);
            add_body(h, SHAPE_CIRCLE, random_color(), BODY_DYNAMIC,
                     r, 0, 0, zero, zero, zero);
        }

        /* ---------- F: 对所有动态物体施加向上冲量 ---------- */
        if (IsKeyPressed(KEY_F)) {
            for (int i = 0; i < g_body_count; i++) {
                if (g_bodies[i].body_type == BODY_DYNAMIC) {
                    rp_body_apply_impulse(g_world, g_bodies[i].handle, 0, 8.0f);
                }
            }
        }

        /* ---------- G: 切换重力 ---------- */
        if (IsKeyPressed(KEY_G)) {
            static bool gravity_on = true;
            gravity_on = !gravity_on;
            if (gravity_on) {
                rp_world_set_gravity(g_world, 0.0f, -9.81f);
            } else {
                rp_world_set_gravity(g_world, 0.0f, 0.0f);
            }
        }

        /* ---------- 拖拽模式 (按住 D + 左键) ---------- */
        if (IsKeyDown(KEY_D)) {
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                RpHandle hit = rp_query_point_intersect(g_world, mx, my, ALL_GROUPS);
                int idx = find_body_index(hit);
                if (idx >= 0) {
                    RpVec2 pos = rp_collider_get_position(g_world, hit);
                    g_drag = (DragState){
                        .active = true, .body_idx = idx,
                        .offset_x = pos.x - mx, .offset_y = pos.y - my,
                    };
                    /* 拖拽时临时变为 Kinematic 使其不受重力影响 */
                    rp_body_set_type(g_world, hit, BODY_KINEMATIC);
                    rp_body_set_velocity(g_world, hit, 0, 0);
                }
            }
            if (g_drag.active && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                RpHandle h = g_bodies[g_drag.body_idx].handle;
                rp_collider_set_position(g_world, h,
                                         mx + g_drag.offset_x,
                                         my + g_drag.offset_y);
            }
            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && g_drag.active) {
                /* 释放时恢复原始刚体类型 */
                RpHandle h = g_bodies[g_drag.body_idx].handle;
                rp_body_set_type(g_world, h, g_bodies[g_drag.body_idx].body_type);
                g_drag.active = false;
            }
        } else {
            if (g_drag.active) {
                RpHandle h = g_bodies[g_drag.body_idx].handle;
                rp_body_set_type(g_world, h, g_bodies[g_drag.body_idx].body_type);
                g_drag.active = false;
            }

            /* ---------- 左键: 射线检测 ---------- */
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                float ox = mx, oy = 10.0f;
                float dx = mx - ox, dy = my - oy;
                float len = sqrtf(dx * dx + dy * dy);
                if (len > 0.001f) { dx /= len; dy /= len; }
                g_ray = (RayVis){
                    .active = true, .ox = ox, .oy = oy, .dx = dx, .dy = dy,
                    .hit = rp_query_ray_cast_with_normal(
                        g_world, ox, oy, dx, dy, 100.0f, ALL_GROUPS),
                    .timer = 2.0f,
                };
            }

            /* ---------- 右键: 圆形区域查询 ---------- */
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
                float query_radius = 2.0f;
                for (int i = 0; i < g_body_count; i++)
                    g_bodies[i].query_hit = false;

                RpHandle results[MAX_BODIES];
                uint32_t count = rp_query_intersect_circle(
                    g_world, mx, my, query_radius, ALL_GROUPS,
                    results, MAX_BODIES);

                for (uint32_t i = 0; i < count; i++) {
                    int idx = find_body_index(results[i]);
                    if (idx >= 0) g_bodies[idx].query_hit = true;
                }
                g_query = (QueryVis){
                    .active = true, .x = mx, .y = my,
                    .radius = query_radius, .timer = 2.0f,
                };
            }
        }

        /* ---------- 物理步进 ---------- */
        rp_world_step(g_world);

        /* ---------- 更新计时器 ---------- */
        if (g_ray.active) {
            g_ray.timer -= dt;
            if (g_ray.timer <= 0) g_ray.active = false;
        }
        if (g_query.active) {
            g_query.timer -= dt;
            if (g_query.timer <= 0) {
                g_query.active = false;
                for (int i = 0; i < g_body_count; i++)
                    g_bodies[i].query_hit = false;
            }
        }
        for (int i = 0; i < g_flash_count; ) {
            g_flashes[i].timer -= dt;
            if (g_flashes[i].timer <= 0) {
                g_flashes[i] = g_flashes[--g_flash_count];
            } else {
                i++;
            }
        }

        /* ========== 渲染 ========== */
        BeginDrawing();
        ClearBackground((Color){30, 30, 40, 255});

        /* 网格 */
        for (float wx = -10; wx <= 10; wx += 1.0f) {
            Vector2 top = world_to_screen(wx, 12);
            Vector2 bot = world_to_screen(wx, -10);
            DrawLineV(top, bot, (Color){50, 50, 60, 255});
        }
        for (float wy = -5; wy <= 12; wy += 1.0f) {
            Vector2 left = world_to_screen(-10, wy);
            Vector2 right = world_to_screen(10, wy);
            DrawLineV(left, right, (Color){50, 50, 60, 255});
        }

        /* 坐标轴 */
        {
            Vector2 o = world_to_screen(0, 0);
            Vector2 xend = world_to_screen(1.5f, 0);
            Vector2 yend = world_to_screen(0, 1.5f);
            DrawLineEx(o, xend, 2, RED);
            DrawLineEx(o, yend, 2, GREEN);
            DrawText("X", (int)xend.x + 5, (int)xend.y - 8, 14, RED);
            DrawText("Y", (int)yend.x + 5, (int)yend.y - 8, 14, GREEN);
        }

        /* 碰撞体 */
        for (int i = 0; i < g_body_count; i++) {
            draw_body(&g_bodies[i], g_world);
        }

        /* 射线 */
        draw_ray();

        /* 区域查询 */
        draw_query();

        /* 鼠标十字线 */
        DrawLineV((Vector2){mouse.x - 10, mouse.y},
                  (Vector2){mouse.x + 10, mouse.y}, WHITE);
        DrawLineV((Vector2){mouse.x, mouse.y - 10},
                  (Vector2){mouse.x, mouse.y + 10}, WHITE);

        /* HUD 信息 */
        int y = 10;
        DrawText("rapier2d-ffi Visual Test", 10, y, 20, WHITE); y += 28;
        DrawText(TextFormat("FPS: %d", GetFPS()), 10, y, 16, LIME); y += 22;
        DrawText(TextFormat("Colliders: %u", rp_world_collider_count(g_world)),
                 10, y, 16, LIME); y += 22;
        DrawText(TextFormat("Contact Pairs: %u", rp_world_contact_pair_count(g_world)),
                 10, y, 16, LIME); y += 22;
        DrawText(TextFormat("Mouse: (%.2f, %.2f)", mx, my),
                 10, y, 16, LIGHTGRAY); y += 30;

        DrawText("[LClick] Raycast", 10, y, 14, GREEN); y += 18;
        DrawText("[RClick] Area Query (circle r=2)", 10, y, 14, YELLOW); y += 18;
        DrawText("[D + LClick] Drag collider", 10, y, 14, ORANGE); y += 18;
        DrawText("[Space] Spawn dynamic ball", 10, y, 14, (Color){70, 130, 180, 255}); y += 18;
        DrawText("[F] Launch all dynamic bodies up", 10, y, 14, (Color){186, 85, 211, 255}); y += 18;
        DrawText("[G] Toggle gravity", 10, y, 14, (Color){255, 200, 100, 255}); y += 18;
        DrawText("[R] Reset scene", 10, y, 14, GRAY); y += 18;
        DrawText("[ESC] Quit", 10, y, 14, GRAY);

        /* 射线命中信息 */
        if (g_ray.active && g_ray.hit.hit) {
            int ry = SCREEN_H - 80;
            DrawText("Ray Hit!", SCREEN_W - 200, ry, 16, GREEN); ry += 20;
            DrawText(TextFormat("  toi: %.3f", g_ray.hit.toi),
                     SCREEN_W - 200, ry, 14, WHITE); ry += 18;
            DrawText(TextFormat("  point: (%.2f, %.2f)",
                     g_ray.hit.point.x, g_ray.hit.point.y),
                     SCREEN_W - 200, ry, 14, WHITE); ry += 18;
            DrawText(TextFormat("  normal: (%.2f, %.2f)",
                     g_ray.hit.normal.x, g_ray.hit.normal.y),
                     SCREEN_W - 200, ry, 14, WHITE);
        }

        EndDrawing();
    }

    rp_world_destroy(g_world);
    CloseWindow();
    return 0;
}
