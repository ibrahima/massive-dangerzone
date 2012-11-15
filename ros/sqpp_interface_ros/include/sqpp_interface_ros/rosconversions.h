#include <btBulletCollisionCommon.h>
#include "simulation/simulation_fwd.h"
#include <moveit/collision_detection/collision_world.h>

KinBody moveitObjectToKinBody(collision_detection::Object object);

void importCollisionWorld(Environment::Ptr env, RaveInstance::Ptr rave, collision_detection::CollisionWorld& world);
