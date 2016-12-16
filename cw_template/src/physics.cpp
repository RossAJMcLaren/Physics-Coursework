#include "physics.h"
#include "cPhysicsComponents.h"
#include "collision.h"

#include <glm/glm.hpp>
using namespace std;
using namespace glm;
static vector<cParticle *> physicsScene;
static vector<cCollider *> colliders;

static dvec3 gravity = dvec3(0, -10.0, 0);
const double coef = 0.5;
const double rigidcoef = 0.5;

void ResolveRB(cRigidBody *const b, const collisionInfo &ci, bool which) {
  // Very basic resolution of a rigid body.
	//At an extremely basic view a Rigid body is just a particle with volume - so this resolution resolves the linear components as if the object is a particle and then resolves the angular components.
  dvec3 r0 = b->position - ci.position;
  const double w = (which ? 1.0 : -1.0);
  b->position += (ci.normal) * (w * (ci.depth * 0.5));
    const double currentSpeed = glm::length(b->position - b->prev_position);
  b->prev_position = b->position - (ci.normal) * (currentSpeed)* rigidcoef * w;
  // angular impulse
  auto gg = cross(r0, ci.normal);
  b->angVelocity += b->worldInvInertia * cross(r0, ci.normal);

}

void ResolveP(cParticle *const b, const collisionInfo &ci, bool which) {
  const double w = (which ? -1.0 : 1.0);
  b->position += (ci.normal) * (ci.depth * 0.5);
  const double currentSpeed = glm::length(b->position - b->prev_position);
  b->prev_position = b->position +  (w * ci.normal) * currentSpeed * coef;
}

void Resolve(const collisionInfo &ci) {

  const auto r1 = ci.c1->GetParent()->getCompatibleComponent<cRigidBody>();
  const auto r2 = ci.c2->GetParent()->getCompatibleComponent<cRigidBody>();
  const auto p1 = ci.c1->GetParent()->getCompatibleComponent<cParticle>();
  const auto p2 = ci.c2->GetParent()->getCompatibleComponent<cParticle>();

  if (r1 != NULL) {
    ResolveRB(r1, ci, false);
  } else if (p1 != NULL) {
    ResolveP(p1, ci, false);
  }

  if (r2 != NULL) {
    ResolveRB(r2, ci, true);
  } else if (p2 != NULL) {
    ResolveP(p2, ci, true);
  }
}

void UpdatePhysics(const double t, const double dt) {
  std::vector<collisionInfo> collisions;
  // check for collisions
  {
    for (size_t i = 0; i < colliders.size(); ++i) {
      for (size_t j = i + 1; j < colliders.size(); ++j) {
        collision::IsColliding(collisions, *colliders[i], *colliders[j]);
      }
    }
  }
  // handle collisions
  {
    for (auto &c : collisions) {
      Resolve(c);
    }
  }
  // Integrate
  for (auto &e : physicsScene) {
    e->Integrate(dt);
  }
}

std::vector<cParticle *> &GetPhysicsScene() { return physicsScene; }

std::vector<cCollider *> &GetColliders() { return colliders; }

const glm::dvec3 &GetGravity() { return gravity; }

void InitPhysics() {}

void ShutdownPhysics() {}