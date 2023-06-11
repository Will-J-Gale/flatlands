#pragma once
#include <RigidBody.h>
#include <colliders/Collider.h>

struct Entity
{
    std::shared_ptr<RigidBody> rigidBody;
    std::shared_ptr<Collider> collider;
};