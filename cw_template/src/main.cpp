#include "Particle.h"
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <graphics_framework.h>
#include <phys_utils.h>
#include <thread>

using namespace std;
using namespace graphics_framework;
using namespace glm;
#define physics_tick 1.0 / 60.0
class Firework : public Particle
{
public:
	unsigned type;
	double age;

	bool update(double delta_time)
	{
		integrate(delta_time);
		age -= delta_time;
		return (age < 0) || (position.y < 0);
	}
};

struct FireworkRules
{
	const glm::dvec3 gravity = glm::dvec3(0, -9.81, 0);
	unsigned type;
	double age;
	glm::dvec3 velocity;
	double damping;

	struct Payload
	{
		unsigned type;
		unsigned count;

		void set(unsigned type, unsigned count)
		{
			Payload::type = type;
			Payload::count = count;
		}
	};

	unsigned payloadCount;
	Payload *payloads;

	FireworkRules() 
		: 
		payloadCount(0),
		payloads(NULL)
	{
	}

	void init(unsigned payloadCount)
	{
		FireworkRules::payloadCount = payloadCount;
		payloads = new Payload[payloadCount];
	}

	~FireworkRules()
	{
		if (payloads != NULL)
		{
			delete[] payloads;
		}
	}

	void setParameters(unsigned type, double Age,
		const glm::dvec3 &Velocity, double damping)
	{
		FireworkRules::type = type;
		FireworkRules::age = Age;
		FireworkRules::velocity = Velocity;
		FireworkRules::damping = damping;
	}

	void create(Firework *firework, const Firework *parent = NULL) const
	{
		firework->type = type;
		firework->age = age;
		glm::dvec3 fireworkVelocity;

		if (parent) {
			firework->setPosition(parent->getPosition());
			velocity += parent->getVelocity();
		}
		else
		{
			glm::dvec3 startPosition;
			int x = rand() % 3 - 1;
			startPosition.x = 5.0 * x;
			firework->setPosition(startPosition);
		}

		fireworkVelocity += velocity;
		firework->setVelocity(fireworkVelocity);
		firework->setMass(1);
		firework->setDamping(damping);
		firework->setAcceleration(gravity);
		firework->clearAccumulator();
	}
};

class FireworksDemo : public app
{
	const static unsigned maxFireworks = 1024;
};
void main() {
	// Create application
	app application;
	// Set load content, update and render methods
	application.set_load_content(load_content);
	application.set_update(update);
	application.set_render(render);
	// Run application
	application.run();
}