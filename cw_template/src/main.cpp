#include "game.h"
#include "physics.h"
#include "cPhysicsComponents.h"
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <graphics_framework.h>
#include <phys_utils.h>
#include <thread>
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <graphics_framework.h>
#include <phys_utils.h>

using namespace std;
using namespace graphics_framework;
using namespace glm;
#define physics_tick 1.0 / 60.0

static vector<unique_ptr<Entity>> SceneList;
static unique_ptr<Entity> floorEnt;

//Basic rigid body creator taken from tutorial six
unique_ptr<Entity> CreateCar(const vec3 &position, const int identification) {
	unique_ptr<Entity> ent(new Entity());
	std::string name = "Car ";
	name += std::to_string(identification);
	ent->SetPosition(position);
	ent->SetRotation(angleAxis(0.0f, vec3(0, 0, 0)));
	unique_ptr<Component> physComponent(new cRigidCube());
	unique_ptr<cShapeRenderer> renderComponent(new cShapeRenderer(cShapeRenderer::BOX));
	renderComponent->SetColour(phys::RandomColour());
	ent->AddComponent(physComponent);
	ent->SetName(name);
	ent->AddComponent(unique_ptr<Component>(new cBoxCollider()));
	ent->AddComponent(unique_ptr<Component>(move(renderComponent)));

	return ent;
}

bool update(double delta_time) {
	static double t = 0.0;
	static double accumulator = 0.0;
	accumulator += delta_time;

	//Pressing the Space Bar allows the user to collide both cubes
	if (glfwGetKey(renderer::get_window(), GLFW_KEY_SPACE)) {
		for (auto &e : SceneList) {
			auto b = e->getComponent<cRigidCube>();
			if (b->GetParent()->GetName() == "Car 1") {
				b->AddLinearForce(dvec3(200.0, 0.0, 0.0));
			}
			if (b->GetParent()->GetName() == "Car 2") {
				b->AddLinearForce(dvec3(-200.0, 0.0, 0.0));
			}
		}
	}
	// Pressing Q allows the second cube to move and collide into the first
	if (glfwGetKey(renderer::get_window(), GLFW_KEY_Q)) {
		for (auto &e : SceneList) {
			auto b = e->getComponent<cRigidCube>();
			if (b->GetParent()->GetName() == "Car 2") {
				b->AddLinearForce(dvec3(-100.0, 0.0, 0.0));
				b->AddAngularForce(dvec3(50.0, 0.0, 0.0));
			}
		}
	}
	// Pressing E allows the first cube to move and collide into the second
	if (glfwGetKey(renderer::get_window(), GLFW_KEY_E)) {
		for (auto &e : SceneList) {
			auto b = e->getComponent<cRigidCube>();
			if (b->GetParent()->GetName() == "Car 1") {
				b->AddLinearForce(dvec3(100.0, 0.0, 0.0));
				b->AddAngularForce(dvec3(50.0, 0.0, 0.0));
			}
		}
	}

	while (accumulator > physics_tick) {
		UpdatePhysics(t, physics_tick);
		accumulator -= physics_tick;
		t += physics_tick;
	}

	for (auto &e : SceneList) {
		e->Update(delta_time);
	}

	phys::Update(delta_time);
	return true;
}

bool load_content() {
	phys::Init();
	// Creates both collision object
	SceneList.push_back(move(CreateCar({ -4, 4, 0 }, 1)));
	SceneList.push_back(move(CreateCar({ 4, 4, 0 }, 2)));

	floorEnt = unique_ptr<Entity>(new Entity());
	floorEnt->AddComponent(unique_ptr<Component>(new cPlaneCollider()));
	floorEnt->SetName("Floor");
	phys::SetCameraPos(vec3(20.0f, 10.0f, 20.0f));
	phys::SetCameraTarget(vec3(0, 10.0f, 0));
	InitPhysics();
	return true;
}

bool render() {
	for (auto &e : SceneList) {
		e->Render();
	}
	phys::DrawScene();
	return true;
}

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