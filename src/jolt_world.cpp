#include <avernal/physics/jolt.hpp>

#include <avernal/core/assert.hpp>

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <thread>

JPH_SUPPRESS_WARNINGS

namespace avernal {
namespace {

[[nodiscard]] JPH::Vec3 to_jolt(const Vec3& value) {
    return {value.x, value.y, value.z};
}

[[nodiscard]] Vec3 from_jolt(JPH::Vec3Arg value) {
    return {value.GetX(), value.GetY(), value.GetZ()};
}

[[nodiscard]] JPH::RVec3 to_jolt_pos(const Vec3& value) {
    return {value.x, value.y, value.z};
}

[[nodiscard]] Vec3 from_jolt_pos(JPH::RVec3Arg value) {
    return {static_cast<float>(value.GetX()), static_cast<float>(value.GetY()),
        static_cast<float>(value.GetZ())};
}

[[nodiscard]] JPH::Quat to_jolt(const Quat& value) {
    return {value.x, value.y, value.z, value.w};
}

[[nodiscard]] Quat from_jolt(JPH::QuatArg value) {
    return {value.GetX(), value.GetY(), value.GetZ(), value.GetW()};
}

[[nodiscard]] BodyId from_jolt(JPH::BodyID id) {
    return id.IsInvalid() ? BodyId{} : BodyId{id.GetIndexAndSequenceNumber()};
}

[[nodiscard]] JPH::BodyID to_jolt(BodyId id) {
    return id.is_valid() ? JPH::BodyID{id.value()} : JPH::BodyID{};
}

void trace_impl(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    std::vfprintf(stderr, fmt, args);
    va_end(args);
    std::fputc('\n', stderr);
}

#ifdef JPH_ENABLE_ASSERTS
bool assert_failed_impl(const char* expression, const char* message, const char* file, unsigned line) {
    std::fprintf(stderr, "%s:%u: (%s) %s\n", file, line, expression, message != nullptr ? message : "");
    return false;
}
#endif

class JoltRuntime {
public:
    class Guard {
    public:
        Guard() { add_ref(); }
        ~Guard() { release(); }

        Guard(const Guard&) = delete;
        Guard& operator=(const Guard&) = delete;
    };

private:
    static void add_ref() {
        std::lock_guard lock{mutex()};
        if (refcount()++ == 0) {
            JPH::RegisterDefaultAllocator();
            JPH::Trace = trace_impl;
            JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = assert_failed_impl;)
            JPH::Factory::sInstance = new JPH::Factory();
            JPH::RegisterTypes();
        }
    }

    static void release() {
        std::lock_guard lock{mutex()};
        if (--refcount() == 0) {
            JPH::UnregisterTypes();
            delete JPH::Factory::sInstance;
            JPH::Factory::sInstance = nullptr;
        }
    }

    [[nodiscard]] static std::mutex& mutex() {
        static std::mutex instance;
        return instance;
    }

    [[nodiscard]] static int& refcount() {
        static int count{};
        return count;
    }
};

namespace Layers {
constexpr JPH::ObjectLayer non_moving = 0;
constexpr JPH::ObjectLayer moving = 1;
constexpr JPH::ObjectLayer count = 2;
}  // namespace Layers

namespace BroadPhaseLayers {
constexpr JPH::BroadPhaseLayer non_moving{0};
constexpr JPH::BroadPhaseLayer moving{1};
constexpr JPH::uint count{2};
}  // namespace BroadPhaseLayers

class BroadPhaseLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    BroadPhaseLayerInterfaceImpl() {
        object_to_broad_phase_[Layers::non_moving] = BroadPhaseLayers::non_moving;
        object_to_broad_phase_[Layers::moving] = BroadPhaseLayers::moving;
    }

    [[nodiscard]] JPH::uint GetNumBroadPhaseLayers() const override {
        return BroadPhaseLayers::count;
    }

    [[nodiscard]] JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        JPH_ASSERT(layer < Layers::count);
        return object_to_broad_phase_[layer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    [[nodiscard]] const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
        switch (static_cast<JPH::BroadPhaseLayer::Type>(layer)) {
        case static_cast<JPH::BroadPhaseLayer::Type>(BroadPhaseLayers::non_moving):
            return "NON_MOVING";
        case static_cast<JPH::BroadPhaseLayer::Type>(BroadPhaseLayers::moving):
            return "MOVING";
        default:
            JPH_ASSERT(false);
            return "INVALID";
        }
    }
#endif

private:
    JPH::BroadPhaseLayer object_to_broad_phase_[Layers::count]{};
};

class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    [[nodiscard]] bool ShouldCollide(JPH::ObjectLayer layer1, JPH::BroadPhaseLayer layer2) const override {
        switch (layer1) {
        case Layers::non_moving:
            return layer2 == BroadPhaseLayers::moving;
        case Layers::moving:
            return true;
        default:
            JPH_ASSERT(false);
            return false;
        }
    }
};

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    [[nodiscard]] bool ShouldCollide(JPH::ObjectLayer object1, JPH::ObjectLayer object2) const override {
        switch (object1) {
        case Layers::non_moving:
            return object2 == Layers::moving;
        case Layers::moving:
            return true;
        default:
            JPH_ASSERT(false);
            return false;
        }
    }
};

[[nodiscard]] JPH::EMotionType to_jolt(BodyMotion motion) {
    switch (motion) {
    case BodyMotion::static_body:
        return JPH::EMotionType::Static;
    case BodyMotion::kinematic:
        return JPH::EMotionType::Kinematic;
    case BodyMotion::dynamic:
        return JPH::EMotionType::Dynamic;
    }
    return JPH::EMotionType::Static;
}

[[nodiscard]] JPH::ObjectLayer object_layer(BodyMotion motion) {
    return motion == BodyMotion::static_body ? Layers::non_moving : Layers::moving;
}

[[nodiscard]] JPH::RefConst<JPH::Shape> make_shape(const Shape& shape) {
    switch (shape.type) {
    case ShapeType::box:
        return new JPH::BoxShape(to_jolt(shape.half_extents));
    case ShapeType::sphere:
        return new JPH::SphereShape(shape.radius);
    case ShapeType::capsule:
        return new JPH::CapsuleShape(shape.half_height, shape.radius);
    }
    return {};
}

[[nodiscard]] bool height_field_sample_count_ok(std::uint32_t count) noexcept {
    return count >= 4 && count % 2 == 0;
}

[[nodiscard]] JPH::RefConst<JPH::Shape> make_height_field(const HeightFieldShapeDesc& desc) {
    if (!desc.is_valid()) {
        return {};
    }

    if (desc.width == desc.height && height_field_sample_count_ok(desc.width)) {
        JPH::HeightFieldShapeSettings settings(desc.heights.data(), JPH::Vec3::sZero(),
            JPH::Vec3(desc.scale_x, desc.scale_y, desc.scale_z), desc.width);
        const auto result = settings.Create();
        if (result.IsValid()) {
            return result.Get();
        }
    }

    JPH::VertexList vertices;
    vertices.reserve(static_cast<std::size_t>(desc.width) * desc.height);
    for (std::uint32_t z = 0; z < desc.height; ++z) {
        for (std::uint32_t x = 0; x < desc.width; ++x) {
            const float height = desc.heights[static_cast<std::size_t>(z) * desc.width + x];
            vertices.push_back(JPH::Float3{static_cast<float>(x) * desc.scale_x, height * desc.scale_y,
                static_cast<float>(z) * desc.scale_z});
        }
    }

    JPH::IndexedTriangleList triangles;
    triangles.reserve(static_cast<std::size_t>(desc.width - 1) * (desc.height - 1) * 2);
    for (std::uint32_t z = 0; z + 1 < desc.height; ++z) {
        for (std::uint32_t x = 0; x + 1 < desc.width; ++x) {
            const auto i0 = z * desc.width + x;
            const auto i1 = i0 + 1;
            const auto i2 = i0 + desc.width;
            const auto i3 = i2 + 1;
            triangles.push_back(JPH::IndexedTriangle(i0, i2, i1, 0));
            triangles.push_back(JPH::IndexedTriangle(i1, i2, i3, 0));
        }
    }

    JPH::MeshShapeSettings settings(std::move(vertices), std::move(triangles));
    const auto result = settings.Create();
    return result.IsValid() ? result.Get() : JPH::RefConst<JPH::Shape>{};
}

[[nodiscard]] int job_thread_count() {
    const auto hardware = std::thread::hardware_concurrency();
    if (hardware <= 1) {
        return 0;
    }
    return static_cast<int>(hardware - 1);
}

class JoltWorld final : public PhysicsWorld {
public:
    explicit JoltWorld(const PhysicsWorldDesc& desc)
        : temp_allocator_(desc.temp_allocator_bytes),
          job_system_(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, job_thread_count()) {
        AV_ENSURE(desc.is_valid());
        system_.Init(desc.max_bodies, 0, desc.max_body_pairs, desc.max_contact_constraints,
            broad_phase_layers_, object_vs_broad_phase_, object_vs_object_);
        system_.SetGravity(to_jolt(desc.gravity));
    }

    [[nodiscard]] PhysicsBackend backend() const noexcept override {
        return PhysicsBackend::jolt;
    }

    void set_gravity(const Vec3& gravity) override { system_.SetGravity(to_jolt(gravity)); }

    [[nodiscard]] Vec3 gravity() const override { return from_jolt(system_.GetGravity()); }

    void step(float delta_time, int collision_steps) override {
        AV_ENSURE(delta_time > 0.0f);
        AV_ENSURE(collision_steps > 0);
        system_.Update(delta_time, collision_steps, &temp_allocator_, &job_system_);
    }

    [[nodiscard]] BodyId create_body(const BodyDesc& desc) override {
        if (!desc.is_valid()) {
            return BodyId{};
        }

        const auto shape = make_shape(desc.shape);
        if (shape == nullptr) {
            return BodyId{};
        }

        JPH::BodyCreationSettings settings(shape, to_jolt_pos(desc.position), to_jolt(desc.rotation),
            to_jolt(desc.motion), object_layer(desc.motion));
        settings.mFriction = desc.friction;
        settings.mRestitution = desc.restitution;
        if (desc.motion == BodyMotion::dynamic) {
            settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            settings.mMassPropertiesOverride.mMass = desc.mass > 0.0f ? desc.mass : 1.0f;
        }

        auto& bodies = system_.GetBodyInterface();
        const auto activation =
            desc.activate ? JPH::EActivation::Activate : JPH::EActivation::DontActivate;
        const auto id = bodies.CreateAndAddBody(settings, activation);
        if (id.IsInvalid()) {
            return BodyId{};
        }

        if (desc.linear_velocity != Vec3{}) {
            bodies.SetLinearVelocity(id, to_jolt(desc.linear_velocity));
        }
        if (desc.angular_velocity != Vec3{}) {
            bodies.SetAngularVelocity(id, to_jolt(desc.angular_velocity));
        }
        return from_jolt(id);
    }

    [[nodiscard]] BodyId create_height_field(
        const HeightFieldShapeDesc& height_field, const BodyDesc& desc) override {
        if (!height_field.is_valid() || desc.mass < 0.0f || desc.friction < 0.0f ||
            desc.restitution < 0.0f) {
            return BodyId{};
        }

        const auto shape = make_height_field(height_field);
        if (shape == nullptr) {
            return BodyId{};
        }

        auto body = desc;
        body.motion = BodyMotion::static_body;
        JPH::BodyCreationSettings settings(shape, to_jolt_pos(body.position), to_jolt(body.rotation),
            JPH::EMotionType::Static, object_layer(BodyMotion::static_body));
        settings.mFriction = body.friction;
        settings.mRestitution = body.restitution;

        auto& bodies = system_.GetBodyInterface();
        const auto activation =
            body.activate ? JPH::EActivation::Activate : JPH::EActivation::DontActivate;
        const auto id = bodies.CreateAndAddBody(settings, activation);
        return from_jolt(id);
    }

    void destroy_body(BodyId id) override {
        if (!id) {
            return;
        }
        auto& bodies = system_.GetBodyInterface();
        const auto jolt_id = to_jolt(id);
        if (bodies.IsAdded(jolt_id)) {
            bodies.RemoveBody(jolt_id);
        }
        bodies.DestroyBody(jolt_id);
    }

    [[nodiscard]] bool contains(BodyId id) const override {
        return id && system_.GetBodyInterface().IsAdded(to_jolt(id));
    }

    [[nodiscard]] BodyPose pose(BodyId id) const override {
        AV_ENSURE(contains(id));
        const auto jolt_id = to_jolt(id);
        auto& bodies = system_.GetBodyInterface();
        return {
            .position = from_jolt_pos(bodies.GetPosition(jolt_id)),
            .rotation = from_jolt(bodies.GetRotation(jolt_id)),
        };
    }

    void set_pose(BodyId id, const BodyPose& pose, bool activate) override {
        AV_ENSURE(contains(id));
        system_.GetBodyInterface().SetPositionAndRotation(to_jolt(id), to_jolt_pos(pose.position),
            to_jolt(pose.rotation),
            activate ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    }

    [[nodiscard]] Vec3 linear_velocity(BodyId id) const override {
        AV_ENSURE(contains(id));
        return from_jolt(system_.GetBodyInterface().GetLinearVelocity(to_jolt(id)));
    }

    void set_linear_velocity(BodyId id, const Vec3& velocity) override {
        AV_ENSURE(contains(id));
        system_.GetBodyInterface().SetLinearVelocity(to_jolt(id), to_jolt(velocity));
    }

    [[nodiscard]] Vec3 angular_velocity(BodyId id) const override {
        AV_ENSURE(contains(id));
        return from_jolt(system_.GetBodyInterface().GetAngularVelocity(to_jolt(id)));
    }

    void set_angular_velocity(BodyId id, const Vec3& velocity) override {
        AV_ENSURE(contains(id));
        system_.GetBodyInterface().SetAngularVelocity(to_jolt(id), to_jolt(velocity));
    }

    void add_force(BodyId id, const Vec3& force) override {
        AV_ENSURE(contains(id));
        system_.GetBodyInterface().AddForce(to_jolt(id), to_jolt(force));
    }

    void add_torque(BodyId id, const Vec3& torque) override {
        AV_ENSURE(contains(id));
        system_.GetBodyInterface().AddTorque(to_jolt(id), to_jolt(torque));
    }

    void add_impulse(BodyId id, const Vec3& impulse) override {
        AV_ENSURE(contains(id));
        system_.GetBodyInterface().AddImpulse(to_jolt(id), to_jolt(impulse));
    }

    [[nodiscard]] std::optional<RayHit> ray_cast(
        const Vec3& origin, const Vec3& direction, float max_distance) const override {
        const float length_sq =
            direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
        if (length_sq <= 0.0f || max_distance <= 0.0f) {
            return std::nullopt;
        }

        const float inv_length = max_distance / std::sqrt(length_sq);
        const JPH::RRayCast ray{to_jolt_pos(origin), to_jolt(direction * inv_length)};
        JPH::RayCastResult hit;
        if (!system_.GetNarrowPhaseQuery().CastRay(ray, hit)) {
            return std::nullopt;
        }

        JPH::BodyLockRead lock(system_.GetBodyLockInterface(), hit.mBodyID);
        if (!lock.Succeeded()) {
            return std::nullopt;
        }

        const auto point = ray.GetPointOnRay(hit.mFraction);
        const auto normal = lock.GetBody().GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, point);
        return RayHit{
            .body = from_jolt(hit.mBodyID),
            .point = from_jolt_pos(point),
            .normal = from_jolt(normal),
            .fraction = hit.mFraction,
        };
    }

private:
    JoltRuntime::Guard runtime_{};
    JPH::TempAllocatorImpl temp_allocator_;
    JPH::JobSystemThreadPool job_system_;
    BroadPhaseLayerInterfaceImpl broad_phase_layers_{};
    ObjectVsBroadPhaseLayerFilterImpl object_vs_broad_phase_{};
    ObjectLayerPairFilterImpl object_vs_object_{};
    JPH::PhysicsSystem system_{};
};

}  // namespace

std::unique_ptr<PhysicsWorld> create_jolt_world(const PhysicsWorldDesc& desc) {
    return std::make_unique<JoltWorld>(desc);
}

}  // namespace avernal
