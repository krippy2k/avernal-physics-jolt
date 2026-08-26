#include <avernal/physics/jolt.hpp>

#include <print>

int main() {
    auto world = avernal::create_jolt_world();
    [[maybe_unused]] const auto floor = world->create_body({
        .motion = avernal::BodyMotion::static_body,
        .shape = avernal::Shape::box({50.0f, 0.5f, 50.0f}),
        .position = {0.0f, -0.5f, 0.0f},
        .activate = false,
    });
    const auto sphere = world->create_body({
        .shape = avernal::Shape::sphere(0.5f),
        .position = {0.0f, 4.0f, 0.0f},
    });

    std::println("backend = {}", avernal::physics_backend_name(world->backend()));
    for (int step = 0; step < 8; ++step) {
        world->step(1.0f / 60.0f);
        const auto y = world->pose(sphere).position.y;
        std::println("step {:>2}  y = {:.3f}", step, y);
    }
    return 0;
}
