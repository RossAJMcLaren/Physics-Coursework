#include <assert.h>
#include "Particle.h"


void Particle::integrate(double delta_time)
{
	if (inverseMass <= 0.0f) return;

	assert(delta_time > 0.0);

	position += (velocity * delta_time);
	dvec3 resultingAcc = acceleration;
	// A = F * 1/M as normally a = f/m
	resultingAcc += (forces * inverseMass);
	velocity += (resultingAcc * delta_time);
	velocity *= pow(damping, delta_time);
	clearAccumulator();
}

void Particle::setMass(const double mass)
{
	assert(mass != 0);
	Particle::mass = mass;
	Particle::setInverseMass();
}

double Particle::getMass() const
{
	assert(mass != 0);
	return mass;
}

void Particle::setInverseMass()
{
	Particle::inverseMass = (1.0 / mass);
}

double Particle::getInverseMass() const
{
	assert(inverseMass != 0);
	return inverseMass;
}

void Particle::setDamping(const double damping)
{
	Particle::damping = damping;
}

double Particle::getDamping() const
{
	return damping;
}

void Particle::setPosition(const dvec3 &position)
{
	Particle::position = position;
}

void Particle::setPosition(const double x, const double y, const double z)
{
	position.x = x;
	position.y = y;
	position.z = z;
}

dvec3 Particle::getPosition() const
{
	return position;
}

void Particle::setVelocity(const dvec3 &velocity)
{
	Particle::velocity = velocity;
}

void Particle::setVelocity(const double x, const double y, const double z)
{
	velocity.x = x;
	velocity.y = y;
	velocity.z = z;
}

dvec3 Particle::getVelocity() const
{
	return velocity;
}

void Particle::setAcceleration(const dvec3 &acceleration)
{
	Particle::acceleration = acceleration;
}

void Particle::setAcceleration(const double x, const double y, const double z)
{
	acceleration.x = x;
	acceleration.y = y;
	acceleration.z = z;
}

dvec3 Particle::getAcceleration() const
{
	return acceleration;
}

void Particle::clearAccumulator()
{
	forces.x = 0;
	forces.y = 0;
	forces.z = 0;
}

void Particle::addForce(const dvec3 &force)
{
	forces += force;
}