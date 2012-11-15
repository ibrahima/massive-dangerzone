#include <btBulletCollisionCommon.h>
#include "simulation/simulation_fwd.h"
#include <moveit/collision_detection/collision_world.h>
#include "simulation/environment.h"
#include "simulation/openravesupport.h"
#include "simulation/simplescene.h"
#include "sqp/traj_sqp.h"
#include "sqp/collisions.h"
#include "sqp/planning_problems2.h"

OpenRAVE::KinBody moveitObjectToKinBody(collision_detection::CollisionWorld::Object object);

void importCollisionWorld(Environment::Ptr env, RaveInstance::Ptr rave, const collision_detection::CollisionWorldConstPtr world);
