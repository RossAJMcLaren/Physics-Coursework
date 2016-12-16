#pragma once
#include <glm/glm.hpp>

using namespace std;
using namespace glm;

class Particle
{
public:

protected:
	dvec3 velocity;
	dvec3 position;
	dvec3 acceleration;
	dvec3 forces;
	double mass;
	double inverseMass;
	double damping;

public:
	void integrate(double delta_time);
	void setMass(double mass);
	double getMass() const;
	double getInverseMass() const;
	void setDamping(const double damping);
	double getDamping() const;
	void setPosition(const dvec3 &position);
	void setPosition(const double x, const double y, const double z);
	dvec3 getPosition() const;
	void setVelocity(const dvec3 &velocity);
	void setVelocity(const double x, const double y, const double z);
	dvec3 getVelocity() const;
	void setAcceleration(const dvec3 &acceleration);
	void setAcceleration(const double x, const double y, const double z);
	dvec3 getAcceleration() const;
	void clearAccumulator();
	void addForce(const dvec3 &force);

protected:
	void setInverseMass();
};