/**
 * rapier2d-ffi test suite
 *
 * Build:
 *   cargo build --release
 *   cc -o test/test_rapier2d test/test_rapier2d.c \
 *      -I. -Ltarget/release -lrapier2d_ffi \
 *      -framework Security -framework CoreFoundation   # macOS only
 *
 * Run:
 *   DYLD_LIBRARY_PATH=target/release ./test/test_rapier2d     # macOS
 *   LD_LIBRARY_PATH=target/release   ./test/test_rapier2d     # Linux
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "rapier2d_ffi.h"

/* ------------------------------------------------------------------ */
/*  tiny test helpers                                                  */
/* ------------------------------------------------------------------ */

static int g_total   = 0;
static int g_passed  = 0;
static int g_failed  = 0;

#define ASSERT(expr)                                                         \
    do {                                                                     \
        g_total++;                                                           \
        if (expr) { g_passed++; }                                            \
        else {                                                               \
            g_failed++;                                                      \
            printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, #expr);       \
        }                                                                    \
    } while (0)

#define ASSERT_FLOAT_EQ(a, b)  ASSERT(fabsf((a) - (b)) < 1e-3f)

#define RUN_TEST(fn)                                                         \
    do {                                                                     \
        printf("-- %s\n", #fn);                                              \
        fn();                                                                \
    } while (0)

/* ------------------------------------------------------------------ */
/*  callback state                                                     */
/* ------------------------------------------------------------------ */

static int g_collision_start_count = 0;
static int g_collision_stop_count  = 0;
static int g_log_count             = 0;

static void on_collision_start(RpHandle a, RpHandle b) {
    (void)a; (void)b;
    g_collision_start_count++;
}

static void on_collision_stop(RpHandle a, RpHandle b) {
    (void)a; (void)b;
    g_collision_stop_count++;
}

static void on_log(const char *msg) {
    (void)msg;
    g_log_count++;
}

/* ------------------------------------------------------------------ */
/*  1. World lifecycle                                                 */
/* ------------------------------------------------------------------ */

static void test_world_lifecycle(void) {
    RpWorld *w = rp_world_create(0.0f, -9.81f, NULL, NULL);
    ASSERT(w != NULL);

    /* step should not crash on an empty world */
    rp_world_step(w);
    ASSERT(rp_world_collider_count(w) == 0);

    rp_world_destroy(w);
    /* calling destroy on NULL should be safe */
    rp_world_destroy(NULL);
}

/* ------------------------------------------------------------------ */
/*  2. World settings                                                  */
/* ------------------------------------------------------------------ */

static void test_world_settings(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    rp_world_set_gravity(w, 0.0f, -20.0f);
    rp_world_set_timestep(w, 1.0f / 120.0f);
    /* no getter yet — just make sure it doesn't crash */
    rp_world_step(w);

    /* NULL world should not crash */
    rp_world_set_gravity(NULL, 0.0f, 0.0f);
    rp_world_set_timestep(NULL, 0.0f);
    rp_world_step(NULL);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  3. Collider creation — every shape                                 */
/* ------------------------------------------------------------------ */

static void test_collider_create_shapes(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    /* circle */
    RpHandle circle = rp_collider_create_circle(w, 0, 0, 1.0f, 0, 0, 0xFFFFFFFF);
    ASSERT(rp_handle_is_valid(w, circle));

    /* rect */
    RpHandle rect = rp_collider_create_rect(w, 1, 2, 0.5f, 2.0f, 1.0f, 0, 0, 0xFFFFFFFF);
    ASSERT(rp_handle_is_valid(w, rect));

    /* triangle */
    RpVec2 ta = {0, 0}, tb = {1, 0}, tc = {0.5f, 1};
    RpHandle tri = rp_collider_create_triangle(w, 3, 4, 0, ta, tb, tc, 0, 0, 0xFFFFFFFF);
    ASSERT(rp_handle_is_valid(w, tri));

    /* convex polygon (pentagon) */
    RpVec2 penta[5] = {
        {0, 1}, {0.95f, 0.31f}, {0.59f, -0.81f}, {-0.59f, -0.81f}, {-0.95f, 0.31f}
    };
    RpHandle conv = rp_collider_create_convex(w, 5, 6, 0, penta, 5, 0, 0, 0xFFFFFFFF);
    ASSERT(rp_handle_is_valid(w, conv));

    /* capsule */
    RpHandle cap = rp_collider_create_capsule(w, 7, 8, 0.3f, 1.0f, 0.5f, 0, 0, 0xFFFFFFFF);
    ASSERT(rp_handle_is_valid(w, cap));

    /* segment */
    RpVec2 sa = {-1, 0}, sb = {1, 0};
    RpHandle seg = rp_collider_create_segment(w, 9, 10, 0, sa, sb, 0, 0, 0xFFFFFFFF);
    ASSERT(rp_handle_is_valid(w, seg));

    ASSERT(rp_world_collider_count(w) == 6);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  4. Convex hull failure                                             */
/* ------------------------------------------------------------------ */

static void test_convex_hull_failure(void) {
    rp_register_log_callback(on_log);
    g_log_count = 0;

    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    /* too few vertices (< 3) should be rejected before calling rapier */
    RpVec2 two[2] = {{0,0},{1,1}};
    RpHandle bad1 = rp_collider_create_convex(w, 0, 0, 0, two, 2, 0, 0, 0xFFFFFFFF);
    ASSERT(!rp_handle_is_valid(w, bad1));

    /* collinear points may fail convex hull */
    RpVec2 collinear[3] = {{0,0},{1,0},{2,0}};
    RpHandle bad2 = rp_collider_create_convex(w, 0, 0, 0, collinear, 3, 0, 0, 0xFFFFFFFF);
    ASSERT(!rp_handle_is_valid(w, bad2));
    /* log callback should have been invoked at least once */
    ASSERT(g_log_count > 0);

    /* NULL pointer should be rejected */
    RpHandle bad3 = rp_collider_create_convex(w, 0, 0, 0, NULL, 5, 0, 0, 0xFFFFFFFF);
    ASSERT(!rp_handle_is_valid(w, bad3));

    rp_register_log_callback(NULL);
    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  5. Collider destroy                                                */
/* ------------------------------------------------------------------ */

static void test_collider_destroy(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    RpHandle c = rp_collider_create_circle(w, 0, 0, 1, 0, 0, 0xFFFFFFFF);
    ASSERT(rp_handle_is_valid(w, c));
    ASSERT(rp_world_collider_count(w) == 1);

    ASSERT(rp_collider_destroy(w, c) == true);
    ASSERT(!rp_handle_is_valid(w, c));
    ASSERT(rp_world_collider_count(w) == 0);

    /* double destroy should return false */
    ASSERT(rp_collider_destroy(w, c) == false);

    /* destroy on NULL world should not crash */
    ASSERT(rp_collider_destroy(NULL, c) == false);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  6. Position / rotation / pose                                      */
/* ------------------------------------------------------------------ */

static void test_position_rotation_pose(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);
    RpHandle c = rp_collider_create_circle(w, 1, 2, 0.5f, 0, 0, 0xFFFFFFFF);

    /* initial position */
    RpVec2 pos = rp_collider_get_position(w, c);
    ASSERT_FLOAT_EQ(pos.x, 1.0f);
    ASSERT_FLOAT_EQ(pos.y, 2.0f);

    /* set position */
    rp_collider_set_position(w, c, 10, 20);
    pos = rp_collider_get_position(w, c);
    ASSERT_FLOAT_EQ(pos.x, 10.0f);
    ASSERT_FLOAT_EQ(pos.y, 20.0f);

    /* set rotation */
    rp_collider_set_rotation(w, c, 1.57f);
    ASSERT_FLOAT_EQ(rp_collider_get_rotation(w, c), 1.57f);
    /* position should not change */
    pos = rp_collider_get_position(w, c);
    ASSERT_FLOAT_EQ(pos.x, 10.0f);
    ASSERT_FLOAT_EQ(pos.y, 20.0f);

    /* set pose (position + rotation together) */
    rp_collider_set_pose(w, c, -5, -6, 0.5f);
    pos = rp_collider_get_position(w, c);
    ASSERT_FLOAT_EQ(pos.x, -5.0f);
    ASSERT_FLOAT_EQ(pos.y, -6.0f);
    ASSERT_FLOAT_EQ(rp_collider_get_rotation(w, c), 0.5f);

    /* invalid handle should return defaults */
    RpHandle invalid = {0xFFFFFFFF, 0xFFFFFFFF};
    pos = rp_collider_get_position(w, invalid);
    ASSERT_FLOAT_EQ(pos.x, 0.0f);
    ASSERT_FLOAT_EQ(pos.y, 0.0f);
    ASSERT_FLOAT_EQ(rp_collider_get_rotation(w, invalid), 0.0f);

    /* NULL world should return defaults */
    pos = rp_collider_get_position(NULL, c);
    ASSERT_FLOAT_EQ(pos.x, 0.0f);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  7. Enable / disable                                                */
/* ------------------------------------------------------------------ */

static void test_enable_disable(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);
    RpHandle c = rp_collider_create_circle(w, 0, 0, 1, 0, 0, 0xFFFFFFFF);

    ASSERT(rp_collider_is_enabled(w, c) == true);

    rp_collider_set_enabled(w, c, false);
    ASSERT(rp_collider_is_enabled(w, c) == false);

    rp_collider_set_enabled(w, c, true);
    ASSERT(rp_collider_is_enabled(w, c) == true);

    /* NULL safety */
    ASSERT(rp_collider_is_enabled(NULL, c) == false);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  8. Collider type (solid / sensor)                                  */
/* ------------------------------------------------------------------ */

static void test_collider_type(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    /* solid by default (collider_type=0) */
    RpHandle solid = rp_collider_create_circle(w, 0, 0, 1, 0, 0, 0xFFFFFFFF);
    ASSERT(rp_collider_get_type(w, solid) == 0);

    /* sensor from creation (collider_type=1) */
    RpHandle sensor = rp_collider_create_circle(w, 5, 5, 1, 1, 0, 0xFFFFFFFF);
    ASSERT(rp_collider_get_type(w, sensor) == 1);

    /* toggle solid -> sensor */
    rp_collider_set_type(w, solid, 1);
    ASSERT(rp_collider_get_type(w, solid) == 1);

    /* toggle sensor -> solid */
    rp_collider_set_type(w, solid, 0);
    ASSERT(rp_collider_get_type(w, solid) == 0);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  9. Collision groups                                                 */
/* ------------------------------------------------------------------ */

static void test_collision_groups(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    uint32_t GROUP_A = 1;
    uint32_t GROUP_B = 2;

    RpHandle a = rp_collider_create_circle(w, 0, 0, 1, 0, 0, GROUP_A);
    RpHandle b = rp_collider_create_circle(w, 0, 0, 1, 0, 0, GROUP_B);

    ASSERT(rp_collider_get_group(w, a) == GROUP_A);
    ASSERT(rp_collider_get_group(w, b) == GROUP_B);

    /* change group */
    rp_collider_set_group(w, a, GROUP_B);
    ASSERT(rp_collider_get_group(w, a) == GROUP_B);

    /* raycast should filter by group */
    RpHandle target = rp_collider_create_circle(w, 10, 0, 1, 0, 0, GROUP_A);

    /* step so broad phase is updated after group changes */
    rp_world_step(w);

    /* ray with GROUP_A filter should hit target but miss a (now GROUP_B) */
    RpRayHit hit = rp_query_ray_cast(w, -5, 0, 1, 0, 100, GROUP_A);
    ASSERT(hit.hit == true);
    ASSERT(hit.handle.id == target.id);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  10. Intersection test (pair)                                       */
/* ------------------------------------------------------------------ */

static void test_intersection_pair(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    /* overlapping circles */
    RpHandle a = rp_collider_create_circle(w, 0, 0, 1, 0, 0, 0xFFFFFFFF);
    RpHandle b = rp_collider_create_circle(w, 0.5f, 0, 1, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);
    ASSERT(rp_query_intersection_test(w, a, b) == true);

    /* non-overlapping circles */
    RpHandle c = rp_collider_create_circle(w, 100, 100, 1, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);
    ASSERT(rp_query_intersection_test(w, a, c) == false);

    /* invalid handle */
    RpHandle invalid = {0xFFFFFFFF, 0xFFFFFFFF};
    ASSERT(rp_query_intersection_test(w, a, invalid) == false);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  11. Point intersect                                                */
/* ------------------------------------------------------------------ */

static void test_point_intersect(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    RpHandle c = rp_collider_create_circle(w, 0, 0, 2, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);

    /* point inside */
    RpHandle hit = rp_query_point_intersect(w, 0.5f, 0.5f, 0xFFFFFFFF);
    ASSERT(rp_handle_is_valid(w, hit));
    ASSERT(hit.id == c.id);

    /* point outside */
    hit = rp_query_point_intersect(w, 100, 100, 0xFFFFFFFF);
    ASSERT(!rp_handle_is_valid(w, hit));

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  12. Raycast                                                        */
/* ------------------------------------------------------------------ */

static void test_raycast(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    /* place a circle at (5,0) with radius 1 */
    RpHandle c = rp_collider_create_circle(w, 5, 0, 1, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);

    /* ray from origin going right */
    RpRayHit hit = rp_query_ray_cast(w, 0, 0, 1, 0, 100, 0xFFFFFFFF);
    ASSERT(hit.hit == true);
    ASSERT(hit.handle.id == c.id);
    ASSERT_FLOAT_EQ(hit.toi, 4.0f); /* edge at x=4 */

    /* ray going left should miss */
    RpRayHit miss = rp_query_ray_cast(w, 0, 0, -1, 0, 100, 0xFFFFFFFF);
    ASSERT(miss.hit == false);

    /* ray with short max_toi should miss */
    miss = rp_query_ray_cast(w, 0, 0, 1, 0, 1.0f, 0xFFFFFFFF);
    ASSERT(miss.hit == false);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  13. Raycast with normal                                            */
/* ------------------------------------------------------------------ */

static void test_raycast_with_normal(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    rp_collider_create_circle(w, 5, 0, 1, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);

    RpRayHitWithNormal hit = rp_query_ray_cast_with_normal(w, 0, 0, 1, 0, 100, 0xFFFFFFFF);
    ASSERT(hit.hit == true);
    ASSERT_FLOAT_EQ(hit.toi, 4.0f);
    /* hit point should be on the left edge of the circle: (4, 0) */
    ASSERT_FLOAT_EQ(hit.point.x, 4.0f);
    ASSERT_FLOAT_EQ(hit.point.y, 0.0f);
    /* normal should point left: (-1, 0) */
    ASSERT_FLOAT_EQ(hit.normal.x, -1.0f);
    ASSERT_FLOAT_EQ(hit.normal.y, 0.0f);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  14. Raycast all                                                    */
/* ------------------------------------------------------------------ */

static void test_raycast_all(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    /* three circles in a line */
    rp_collider_create_circle(w, 3, 0, 0.5f, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, 6, 0, 0.5f, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, 9, 0, 0.5f, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);

    RpHandle hits[16];
    uint32_t count = rp_query_ray_cast_all(w, 0, 0, 1, 0, 100, 0xFFFFFFFF, hits, 16);
    ASSERT(count == 3);

    /* with small buffer, should cap at max_results */
    count = rp_query_ray_cast_all(w, 0, 0, 1, 0, 100, 0xFFFFFFFF, hits, 2);
    ASSERT(count == 2);

    /* NULL buffer should return 0 */
    count = rp_query_ray_cast_all(w, 0, 0, 1, 0, 100, 0xFFFFFFFF, NULL, 16);
    ASSERT(count == 0);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  15. Shape cast (circle sweep)                                      */
/* ------------------------------------------------------------------ */

static void test_shape_cast_circle(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    /* a wall at x=10 */
    rp_collider_create_rect(w, 10, 0, 0, 0.5f, 5, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);

    /* sweep a circle (radius=1) from origin going right */
    RpRayHit hit = rp_query_shape_cast_circle(w, 0, 0, 1, 0, 100, 1.0f, 0xFFFFFFFF);
    ASSERT(hit.hit == true);
    /* should hit at toi ~ 8.5 (wall edge at 9.5, minus circle radius 1) */
    ASSERT_FLOAT_EQ(hit.toi, 8.5f);

    /* sweep in opposite direction should miss */
    RpRayHit miss = rp_query_shape_cast_circle(w, 0, 0, -1, 0, 100, 1.0f, 0xFFFFFFFF);
    ASSERT(miss.hit == false);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  16. Shape cast circle with normal                                  */
/* ------------------------------------------------------------------ */

static void test_shape_cast_circle_with_normal(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    /* a wall at x=10 */
    rp_collider_create_rect(w, 10, 0, 0, 0.5f, 5, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);

    /* sweep a circle (radius=1) from origin going right */
    RpRayHitWithNormal hit = rp_query_shape_cast_circle_with_normal(w, 0, 0, 1, 0, 100, 1.0f, 0xFFFFFFFF);
    ASSERT(hit.hit == true);
    /* should hit at toi ~ 8.5 (wall edge at 9.5, minus circle radius 1) */
    ASSERT_FLOAT_EQ(hit.toi, 8.5f);
    /* normal should point left (away from the wall): (-1, 0) */
    ASSERT_FLOAT_EQ(hit.normal.x, -1.0f);
    ASSERT_FLOAT_EQ(hit.normal.y, 0.0f);

    /* sweep in opposite direction should miss */
    RpRayHitWithNormal miss = rp_query_shape_cast_circle_with_normal(w, 0, 0, -1, 0, 100, 1.0f, 0xFFFFFFFF);
    ASSERT(miss.hit == false);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  16b. Shape cast — stop_at_penetration=false (penetration escape)   */
/* ------------------------------------------------------------------ */

static void test_shape_cast_penetration_escape(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    /*
     * Test stop_at_penetration=false behaviour:
     * When two colliders overlap and we shape-cast the circle UPWARD (perpendicular
     * to the overlap axis), with stop_at_penetration=false the cast should NOT
     * be blocked at toi=0 — allowing the shape to escape the overlap.
     */

    /* Two circles overlapping at origin: circle A at (0,0) r=1, circle B at (0.5,0) r=1 */
    rp_collider_create_circle(w, 0.5f, 0, 1.0f, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);

    /* Cast a circle (r=1) from origin going UP — perpendicular to overlap.
       With stop_at_penetration=false, the initial overlap is ignored,
       so the cast should NOT report a hit (nothing above to hit). */
    RpRayHit hit = rp_query_shape_cast_circle(w, 0, 0, 0, 1, 100, 1.0f, 0xFFFFFFFF);
    ASSERT(hit.hit == false);

    /* same test for the with_normal variant */
    RpRayHitWithNormal hit2 = rp_query_shape_cast_circle_with_normal(w, 0, 0, 0, 1, 100, 1.0f, 0xFFFFFFFF);
    ASSERT(hit2.hit == false);

    /* Non-penetrating case should still work normally:
       cast from far left going right toward the circle at (0.5, 0) */
    RpRayHit normal_hit = rp_query_shape_cast_circle(w, -10, 0, 1, 0, 100, 0.5f, 0xFFFFFFFF);
    ASSERT(normal_hit.hit == true);
    ASSERT(normal_hit.toi > 0.0f);

    RpRayHitWithNormal normal_hit2 = rp_query_shape_cast_circle_with_normal(w, -10, 0, 1, 0, 100, 0.5f, 0xFFFFFFFF);
    ASSERT(normal_hit2.hit == true);
    ASSERT(normal_hit2.toi > 0.0f);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  17. Intersect circle (area query)                                  */
/* ------------------------------------------------------------------ */

static void test_intersect_circle(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    rp_collider_create_circle(w, 0, 0, 1, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, 3, 0, 1, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, 100, 0, 1, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);

    RpHandle hits[16];

    /* large search circle should find the two nearby ones */
    uint32_t count = rp_query_intersect_circle(w, 1.5f, 0, 3.0f, 0xFFFFFFFF, hits, 16);
    ASSERT(count == 2);

    /* tiny search circle far away should find none */
    count = rp_query_intersect_circle(w, -50, -50, 0.1f, 0xFFFFFFFF, hits, 16);
    ASSERT(count == 0);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  17. Intersect rect (area query)                                    */
/* ------------------------------------------------------------------ */

static void test_intersect_rect(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    rp_collider_create_circle(w, 0, 0, 0.5f, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, 2, 0, 0.5f, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, 0, 2, 0.5f, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);

    RpHandle hits[16];

    /* large rect covering everything */
    uint32_t count = rp_query_intersect_rect(w, 1, 1, 0, 5, 5, 0xFFFFFFFF, hits, 16);
    ASSERT(count == 3);

    /* small rect covering only origin */
    count = rp_query_intersect_rect(w, 0, 0, 0, 0.6f, 0.6f, 0xFFFFFFFF, hits, 16);
    ASSERT(count == 1);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  18. Intersect capsule (area query)                                 */
/* ------------------------------------------------------------------ */

static void test_intersect_capsule(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    rp_collider_create_circle(w, 0, -2, 0.5f, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, 0,  0, 0.5f, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, 0,  2, 0.5f, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, 50, 0, 0.5f, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);

    RpHandle hits[16];

    /* tall capsule along Y-axis, should catch the 3 nearby ones */
    uint32_t count = rp_query_intersect_capsule(w, 0, 0, 0, 2.0f, 1.0f, 0xFFFFFFFF, hits, 16);
    ASSERT(count == 3);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  19. Contact pairs                                                  */
/* ------------------------------------------------------------------ */

static void test_contact_pairs(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    /* two overlapping circles */
    rp_collider_create_circle(w, 0, 0, 1, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, 0.5f, 0, 1, 0, 0, 0xFFFFFFFF);
    /* one far away */
    rp_collider_create_circle(w, 100, 100, 1, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);

    uint32_t pair_count = rp_world_contact_pair_count(w);
    ASSERT(pair_count >= 1);

    RpContactPairInfo pairs[16];
    uint32_t written = rp_world_get_contact_pairs(w, pairs, 16);
    ASSERT(written >= 1);

    /* at least one pair should have an active contact */
    bool found_active = false;
    for (uint32_t i = 0; i < written; i++) {
        if (pairs[i].has_any_active_contact) {
            found_active = true;
            break;
        }
    }
    ASSERT(found_active);

    /* NULL buffer should return 0 */
    ASSERT(rp_world_get_contact_pairs(w, NULL, 16) == 0);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  20. Collision event callbacks                                      */
/* ------------------------------------------------------------------ */

static void test_collision_callbacks(void) {
    g_collision_start_count = 0;
    g_collision_stop_count  = 0;

    RpWorld *w = rp_world_create(0.0f, 0.0f, on_collision_start, on_collision_stop);

    /* create two overlapping colliders */
    RpHandle a = rp_collider_create_circle(w, 0, 0, 1, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, 0.5f, 0, 1, 0, 0, 0xFFFFFFFF);
    rp_world_step(w);
    ASSERT(g_collision_start_count >= 1);

    /* move one far away and step — should get a stop event */
    int prev_stop = g_collision_stop_count;
    rp_collider_set_position(w, a, 1000, 1000);
    rp_world_step(w);
    ASSERT(g_collision_stop_count > prev_stop);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  21. Log callback                                                   */
/* ------------------------------------------------------------------ */

static void test_log_callback(void) {
    g_log_count = 0;
    rp_register_log_callback(on_log);

    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    /* trigger a log message by creating a convex with collinear points */
    RpVec2 collinear[3] = {{0,0},{1,0},{2,0}};
    rp_collider_create_convex(w, 0, 0, 0, collinear, 3, 0, 0, 0xFFFFFFFF);
    ASSERT(g_log_count >= 1);

    /* disable logging */
    rp_register_log_callback(NULL);
    int prev = g_log_count;
    rp_collider_create_convex(w, 0, 0, 0, collinear, 3, 0, 0, 0xFFFFFFFF);
    ASSERT(g_log_count == prev);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  22. Group filtering on area queries                                */
/* ------------------------------------------------------------------ */

static void test_group_filtering_queries(void) {
    RpWorld *w = rp_world_create(0.0f, 0.0f, NULL, NULL);

    uint32_t GA = 1, GB = 2;

    rp_collider_create_circle(w, 0, 0, 1, 0, 0, GA);
    rp_collider_create_circle(w, 1, 0, 1, 0, 0, GA);
    rp_collider_create_circle(w, 0, 1, 1, 0, 0, GB);
    rp_world_step(w);

    RpHandle hits[16];

    /* intersect_circle with GA filter: should find 2 */
    uint32_t count = rp_query_intersect_circle(w, 0, 0, 5, GA, hits, 16);
    ASSERT(count == 2);

    /* intersect_circle with GB filter: should find 1 */
    count = rp_query_intersect_circle(w, 0, 0, 5, GB, hits, 16);
    ASSERT(count == 1);

    /* intersect_circle with GA|GB filter: should find all 3 */
    count = rp_query_intersect_circle(w, 0, 0, 5, GA | GB, hits, 16);
    ASSERT(count == 3);

    /* point_intersect with wrong group */
    RpHandle pt = rp_query_point_intersect(w, 0, 0, GB);
    /* (0,0) is inside GB collider at (0,1) with r=1? actually it's at distance 1, borderline */
    /* just test that GA filter finds one */
    pt = rp_query_point_intersect(w, 0, 0, GA);
    ASSERT(rp_handle_is_valid(w, pt));

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  23. find_clear_point determinism                                   */
/* ------------------------------------------------------------------ */

static void test_find_clear_point_determinism(void) {
    RpWorld *w = rp_world_create(0.0f, -9.81f, NULL, NULL);

    /* 放置一些障碍物 */
    rp_collider_create_circle(w, 3.0f, 0.0f, 1.0f, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, -2.0f, 2.0f, 1.5f, 0, 0, 0xFFFFFFFF);
    rp_collider_create_rect(w, 0.0f, -3.0f, 0.0f, 2.0f, 0.5f, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, 1.0f, 4.0f, 0.8f, 1, 0, 0xFFFFFFFF); /* sensor */

    /* ---- mode 0 (随机) 确定性测试：相同参数多次调用结果一致 ---- */
    bool found1, found2;
    RpVec2 p1 = rp_query_find_clear_point(w, 0, 0, 10, 2, 0x3, 0xFFFFFFFF, 0, 200, &found1);
    RpVec2 p2 = rp_query_find_clear_point(w, 0, 0, 10, 2, 0x3, 0xFFFFFFFF, 0, 200, &found2);
    ASSERT(found1 == found2);
    if (found1 && found2) {
        ASSERT_FLOAT_EQ(p1.x, p2.x);
        ASSERT_FLOAT_EQ(p1.y, p2.y);
    }

    /* ---- mode 1 (靠近中心) 确定性测试 ---- */
    RpVec2 p3 = rp_query_find_clear_point(w, 0, 0, 10, 2, 0x3, 0xFFFFFFFF, 1, 200, &found1);
    RpVec2 p4 = rp_query_find_clear_point(w, 0, 0, 10, 2, 0x3, 0xFFFFFFFF, 1, 200, &found2);
    ASSERT(found1 == found2);
    if (found1 && found2) {
        ASSERT_FLOAT_EQ(p3.x, p4.x);
        ASSERT_FLOAT_EQ(p3.y, p4.y);
    }

    /* ---- mode 1 结果应在搜索圆内 ---- */
    if (found1) {
        float dist_mode1 = sqrtf(p3.x * p3.x + p3.y * p3.y);
        ASSERT(dist_mode1 < 10.0f);
    }

    /* ---- 不同参数应得到不同结果（种子不同） ---- */
    RpVec2 p5 = rp_query_find_clear_point(w, 5, 5, 10, 2, 0x3, 0xFFFFFFFF, 0, 200, &found1);
    if (found1 && found2) {
        ASSERT(fabsf(p1.x - p5.x) > 1e-3f || fabsf(p1.y - p5.y) > 1e-3f);
    }

    /* ---- ignore_collider_types 过滤测试 ---- */
    /* 只避开 solid (0x1)，sensor 不影响 */
    rp_query_find_clear_point(w, 0, 0, 10, 2, 0x1, 0xFFFFFFFF, 1, 200, &found1);
    /* 避开全部 (0x3)，包括 sensor */
    rp_query_find_clear_point(w, 0, 0, 10, 2, 0x3, 0xFFFFFFFF, 1, 200, &found2);
    ASSERT(found1); /* 只避 solid 时应该能找到 */

    /* ---- 空世界一定能找到 ---- */
    RpWorld *empty = rp_world_create(0.0f, 0.0f, NULL, NULL);
    bool found_empty;
    RpVec2 pe = rp_query_find_clear_point(empty, 0, 0, 5, 1, 0x3, 0xFFFFFFFF, 0, 10, &found_empty);
    ASSERT(found_empty);
    float de = sqrtf(pe.x * pe.x + pe.y * pe.y);
    ASSERT(de <= 5.0f + 1e-3f);
    rp_world_destroy(empty);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  24. find_clear_point_in_rect                                       */
/* ------------------------------------------------------------------ */

static void test_find_clear_point_in_rect(void) {
    RpWorld *w = rp_world_create(0.0f, -9.81f, NULL, NULL);

    /* 放置一些障碍物 */
    rp_collider_create_circle(w,  3.0f,  2.0f, 1.0f, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w, -2.0f,  1.0f, 1.5f, 0, 0, 0xFFFFFFFF);
    rp_collider_create_rect  (w,  0.0f, -1.0f, 0.0f, 2.0f, 0.5f, 0, 0, 0xFFFFFFFF);
    rp_collider_create_circle(w,  1.0f,  3.0f, 0.8f, 1, 0, 0xFFFFFFFF); /* sensor */

    /* ---- mode 0 确定性：相同参数两次结果相同 ---- */
    bool found1, found2;
    /* 搜索矩形：左下角 (-5, -5)，宽高 10x10 */
    RpVec2 p1 = rp_query_find_clear_point_in_rect(w, -5.0f, -5.0f, 10.0f, 10.0f, 1.5f, 0x3, 0xFFFFFFFF, 0, 200, &found1);
    RpVec2 p2 = rp_query_find_clear_point_in_rect(w, -5.0f, -5.0f, 10.0f, 10.0f, 1.5f, 0x3, 0xFFFFFFFF, 0, 200, &found2);
    ASSERT(found1 == found2);
    if (found1 && found2) {
        ASSERT_FLOAT_EQ(p1.x, p2.x);
        ASSERT_FLOAT_EQ(p1.y, p2.y);
    }

    /* ---- mode 1 确定性 ---- */
    RpVec2 p3 = rp_query_find_clear_point_in_rect(w, -5.0f, -5.0f, 10.0f, 10.0f, 1.5f, 0x3, 0xFFFFFFFF, 1, 200, &found1);
    RpVec2 p4 = rp_query_find_clear_point_in_rect(w, -5.0f, -5.0f, 10.0f, 10.0f, 1.5f, 0x3, 0xFFFFFFFF, 1, 200, &found2);
    ASSERT(found1 == found2);
    if (found1 && found2) {
        ASSERT_FLOAT_EQ(p3.x, p4.x);
        ASSERT_FLOAT_EQ(p3.y, p4.y);
    }

    /* ---- mode 1 结果应在搜索矩形内 ---- */
    if (found1) {
        ASSERT(p3.x >= -5.0f - 1e-3f && p3.x <= 5.0f + 1e-3f);
        ASSERT(p3.y >= -5.0f - 1e-3f && p3.y <= 5.0f + 1e-3f);
    }

    /* ---- mode 0 结果应在搜索矩形内 ---- */
    if (found1) {
        ASSERT(p1.x >= -5.0f - 1e-3f && p1.x <= 5.0f + 1e-3f);
        ASSERT(p1.y >= -5.0f - 1e-3f && p1.y <= 5.0f + 1e-3f);
    }

    /* ---- 不同搜索区域，种子不同，结果不同 ---- */
    bool found3;
    RpVec2 p5 = rp_query_find_clear_point_in_rect(w, 0.0f, 0.0f, 10.0f, 10.0f, 1.5f, 0x3, 0xFFFFFFFF, 0, 200, &found3);
    if (found1 && found3) {
        ASSERT(fabsf(p1.x - p5.x) > 1e-3f || fabsf(p1.y - p5.y) > 1e-3f);
    }

    /* ---- ignore_collider_types: 只避 solid，应仍能找到点 ---- */
    bool found_solid_only;
    rp_query_find_clear_point_in_rect(w, -5.0f, -5.0f, 10.0f, 10.0f, 1.5f, 0x1, 0xFFFFFFFF, 1, 200, &found_solid_only);
    ASSERT(found_solid_only);

    /* ---- 空世界一定能找到，且在矩形内 ---- */
    RpWorld *empty = rp_world_create(0.0f, 0.0f, NULL, NULL);
    bool found_empty;
    RpVec2 pe = rp_query_find_clear_point_in_rect(empty, -3.0f, -3.0f, 6.0f, 6.0f, 0.5f, 0x3, 0xFFFFFFFF, 0, 10, &found_empty);
    ASSERT(found_empty);
    ASSERT(pe.x >= -3.0f - 1e-3f && pe.x <= 3.0f + 1e-3f);
    ASSERT(pe.y >= -3.0f - 1e-3f && pe.y <= 3.0f + 1e-3f);
    rp_world_destroy(empty);

    rp_world_destroy(w);
}

/* ------------------------------------------------------------------ */
/*  25. NULL world safety — no crashes                                 */
/* ------------------------------------------------------------------ */

static void test_null_safety(void) {
    RpHandle h = {0, 0};
    RpVec2 v = {0, 0};
    RpHandle buf[4];
    RpContactPairInfo pbuf[4];

    /* every function should survive a NULL world pointer */
    rp_world_step(NULL);
    rp_world_set_timestep(NULL, 0);
    rp_world_set_gravity(NULL, 0, 0);
    ASSERT(rp_world_collider_count(NULL) == 0);

    ASSERT(!rp_handle_is_valid(NULL, h));
    ASSERT(rp_collider_create_circle(NULL, 0, 0, 1, 0, 0, 0).id == 0xFFFFFFFF);
    ASSERT(rp_collider_create_rect(NULL, 0, 0, 0, 1, 1, 0, 0, 0).id == 0xFFFFFFFF);
    ASSERT(rp_collider_create_triangle(NULL, 0, 0, 0, v, v, v, 0, 0, 0).id == 0xFFFFFFFF);
    ASSERT(rp_collider_create_convex(NULL, 0, 0, 0, &v, 3, 0, 0, 0).id == 0xFFFFFFFF);
    ASSERT(rp_collider_create_capsule(NULL, 0, 0, 0, 1, 1, 0, 0, 0).id == 0xFFFFFFFF);
    ASSERT(rp_collider_create_segment(NULL, 0, 0, 0, v, v, 0, 0, 0).id == 0xFFFFFFFF);
    ASSERT(rp_collider_destroy(NULL, h) == false);

    rp_collider_set_position(NULL, h, 0, 0);
    rp_collider_set_rotation(NULL, h, 0);
    rp_collider_set_pose(NULL, h, 0, 0, 0);
    ASSERT(rp_collider_get_position(NULL, h).x == 0.0f);
    ASSERT(rp_collider_get_rotation(NULL, h) == 0.0f);

    ASSERT(rp_collider_is_enabled(NULL, h) == false);
    rp_collider_set_enabled(NULL, h, true);
    ASSERT(rp_collider_get_type(NULL, h) == 0);
    rp_collider_set_type(NULL, h, 1);
    ASSERT(rp_collider_get_group(NULL, h) == 0);
    rp_collider_set_group(NULL, h, 1);

    RpRayHit rh = rp_query_ray_cast(NULL, 0, 0, 1, 0, 10, 0xFFFFFFFF);
    ASSERT(rh.hit == false);
    RpRayHitWithNormal rhn = rp_query_ray_cast_with_normal(NULL, 0, 0, 1, 0, 10, 0xFFFFFFFF);
    ASSERT(rhn.hit == false);
    ASSERT(rp_query_ray_cast_all(NULL, 0, 0, 1, 0, 10, 0xFFFFFFFF, buf, 4) == 0);
    RpRayHit sc = rp_query_shape_cast_circle(NULL, 0, 0, 1, 0, 10, 1, 0xFFFFFFFF);
    ASSERT(sc.hit == false);
    RpRayHitWithNormal scn = rp_query_shape_cast_circle_with_normal(NULL, 0, 0, 1, 0, 10, 1, 0xFFFFFFFF);
    ASSERT(scn.hit == false);

    ASSERT(rp_query_intersection_test(NULL, h, h) == false);
    ASSERT(!rp_handle_is_valid(NULL, rp_query_point_intersect(NULL, 0, 0, 0xFFFFFFFF)));
    ASSERT(rp_query_intersect_circle(NULL, 0, 0, 1, 0xFFFFFFFF, buf, 4) == 0);
    ASSERT(rp_query_intersect_rect(NULL, 0, 0, 0, 1, 1, 0xFFFFFFFF, buf, 4) == 0);
    ASSERT(rp_query_intersect_capsule(NULL, 0, 0, 0, 1, 1, 0xFFFFFFFF, buf, 4) == 0);

    ASSERT(rp_world_contact_pair_count(NULL) == 0);
    ASSERT(rp_world_get_contact_pairs(NULL, pbuf, 4) == 0);

    bool found = true;
    RpVec2 cp = rp_query_find_clear_point(NULL, 0, 0, 10, 2, 0x3, 0xFFFFFFFF, 0, 100, &found);
    ASSERT(found == false);
    ASSERT(cp.x == 0.0f && cp.y == 0.0f);

    found = true;
    RpVec2 cp2 = rp_query_find_clear_point_in_rect(NULL, -5, -5, 10, 10, 2, 0x3, 0xFFFFFFFF, 0, 100, &found);
    ASSERT(found == false);
    ASSERT(cp2.x == 0.0f && cp2.y == 0.0f);
}

/* ------------------------------------------------------------------ */
/*  main                                                               */
/* ------------------------------------------------------------------ */

int main(void) {
    printf("========================================\n");
    printf("  rapier2d-ffi test suite\n");
    printf("========================================\n\n");

    RUN_TEST(test_world_lifecycle);
    RUN_TEST(test_world_settings);
    RUN_TEST(test_collider_create_shapes);
    RUN_TEST(test_convex_hull_failure);
    RUN_TEST(test_collider_destroy);
    RUN_TEST(test_position_rotation_pose);
    RUN_TEST(test_enable_disable);
    RUN_TEST(test_collider_type);
    RUN_TEST(test_collision_groups);
    RUN_TEST(test_intersection_pair);
    RUN_TEST(test_point_intersect);
    RUN_TEST(test_raycast);
    RUN_TEST(test_raycast_with_normal);
    RUN_TEST(test_raycast_all);
    RUN_TEST(test_shape_cast_circle);
    RUN_TEST(test_shape_cast_circle_with_normal);
    RUN_TEST(test_shape_cast_penetration_escape);
    RUN_TEST(test_intersect_circle);
    RUN_TEST(test_intersect_rect);
    RUN_TEST(test_intersect_capsule);
    RUN_TEST(test_contact_pairs);
    RUN_TEST(test_collision_callbacks);
    RUN_TEST(test_log_callback);
    RUN_TEST(test_group_filtering_queries);
    RUN_TEST(test_find_clear_point_determinism);
    RUN_TEST(test_find_clear_point_in_rect);
    RUN_TEST(test_null_safety);

    printf("\n========================================\n");
    printf("  Total: %d  Passed: %d  Failed: %d\n", g_total, g_passed, g_failed);
    printf("========================================\n");

    return g_failed > 0 ? 1 : 0;
}
