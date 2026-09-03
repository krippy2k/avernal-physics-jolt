#include <avernal/physics/jolt.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace {

[[nodiscard]] std::unique_ptr<avernal::PhysicsWorld> make_world() {
    return avernal::create_jolt_world();
}

}  // namespace

TEST(JoltWorld, ReportsBackend) {
    const auto world = make_world();
    ASSERT_NE(world, nullptr);
    EXPECT_EQ(world->backend(), avernal::PhysicsBackend::jolt);
    EXPECT_FLOAT_EQ(world->gravity().y, -9.81f);
}

TEST(JoltWorld, SphereFallsOntoFloor) {
    auto world = make_world();

    const auto floor = world->create_body({
        .motion = avernal::BodyMotion::static_body,
        .shape = avernal::Shape::box({50.0f, 0.5f, 50.0f}),
        .position = {0.0f, -0.5f, 0.0f},
        .activate = false,
    });
    const auto sphere = world->create_body({
        .motion = avernal::BodyMotion::dynamic,
        .shape = avernal::Shape::sphere(0.5f),
        .position = {0.0f, 4.0f, 0.0f},
        .restitution = 0.0f,
    });

    ASSERT_TRUE(floor);
    ASSERT_TRUE(sphere);

    const float start_y = world->pose(sphere).position.y;
    for (int i = 0; i < 30; ++i) {
        world->step(1.0f / 60.0f);
    }
    EXPECT_LT(world->pose(sphere).position.y, start_y);

    for (int i = 0; i < 150; ++i) {
        world->step(1.0f / 60.0f);
    }

    const float y = world->pose(sphere).position.y;
    EXPECT_GT(y, 0.35f);
    EXPECT_LT(y, 0.75f);
    EXPECT_TRUE(world->contains(floor));
    EXPECT_TRUE(world->contains(sphere));
}

TEST(JoltWorld, RayCastHitsFloor) {
    auto world = make_world();
    const auto floor = world->create_body({
        .motion = avernal::BodyMotion::static_body,
        .shape = avernal::Shape::box({10.0f, 0.5f, 10.0f}),
        .position = {0.0f, -0.5f, 0.0f},
        .activate = false,
    });
    ASSERT_TRUE(floor);

    const auto hit = world->ray_cast({0.0f, 10.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, 20.0f);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->body, floor);
    EXPECT_NEAR(hit->point.y, 0.0f, 0.05f);
    EXPECT_GT(hit->normal.y, 0.5f);
}

TEST(JoltWorld, HeightFieldSupportsRayAndFall) {
    auto world = make_world();

    std::vector<float> heights(16 * 16, 0.0f);
    const auto id = world->create_height_field(
        {
            .width = 16,
            .height = 16,
            .heights = heights,
            .scale_x = 1.0f,
            .scale_z = 1.0f,
        },
        {
            .position = {0.0f, 0.0f, 0.0f},
            .activate = false,
        });
    ASSERT_TRUE(id);

    const auto hit = world->ray_cast({4.0f, 10.0f, 4.0f}, {0.0f, -1.0f, 0.0f}, 20.0f);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->body, id);
    EXPECT_NEAR(hit->point.y, 0.0f, 0.1f);

    const auto sphere = world->create_body({
        .motion = avernal::BodyMotion::dynamic,
        .shape = avernal::Shape::sphere(0.5f),
        .position = {4.0f, 4.0f, 4.0f},
        .restitution = 0.0f,
    });
    ASSERT_TRUE(sphere);
    for (int i = 0; i < 180; ++i) {
        world->step(1.0f / 60.0f);
    }
    EXPECT_GT(world->pose(sphere).position.y, 0.35f);
    EXPECT_LT(world->pose(sphere).position.y, 0.85f);
}

TEST(JoltWorld, DestroyRemovesBody) {
    auto world = make_world();
    const auto id = world->create_body({
        .shape = avernal::Shape::sphere(0.5f),
        .position = {0.0f, 1.0f, 0.0f},
    });
    ASSERT_TRUE(world->contains(id));
    world->destroy_body(id);
    EXPECT_FALSE(world->contains(id));
}

TEST(JoltWorld, SetPoseAndVelocity) {
    auto world = make_world();
    const auto id = world->create_body({
        .motion = avernal::BodyMotion::kinematic,
        .shape = avernal::Shape::box({0.5f, 0.5f, 0.5f}),
        .position = {0.0f, 1.0f, 0.0f},
    });
    ASSERT_TRUE(id);

    world->set_pose(id, {.position = {2.0f, 3.0f, 4.0f}, .rotation = avernal::Quat::identity()});
    EXPECT_FLOAT_EQ(world->pose(id).position.x, 2.0f);
    EXPECT_FLOAT_EQ(world->pose(id).position.y, 3.0f);

    world->set_linear_velocity(id, {1.0f, 2.0f, 3.0f});
    EXPECT_FLOAT_EQ(world->linear_velocity(id).z, 3.0f);
}

TEST(JoltWorld, TorqueChangesAngularVelocity) {
    auto world = make_world();
    const auto id = world->create_body({
        .shape = avernal::Shape::sphere(0.5f),
        .position = {0.0f, 8.0f, 0.0f},
    });
    ASSERT_TRUE(id);

    world->add_torque(id, {0.0f, 12.0f, 0.0f});
    world->step(1.0f / 60.0f);
    EXPECT_GT(world->angular_velocity(id).y, 0.0f);
}
