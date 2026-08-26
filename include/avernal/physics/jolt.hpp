#pragma once

#include <avernal/physics/physics.hpp>

#include <memory>

namespace avernal {

[[nodiscard]] std::unique_ptr<PhysicsWorld> create_jolt_world(const PhysicsWorldDesc& desc = {});

}  // namespace avernal
