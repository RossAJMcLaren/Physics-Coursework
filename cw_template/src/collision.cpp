#include "collision.h"
#include "cPhysicsComponents.h"
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
using namespace std;
using namespace glm;
namespace collision {
	//The check to see if the corners given by OBB are colliding
bool cornerCollision(double position, double minimumPosition, double maximumPosition)
{
	return minimumPosition <= position && position <= maximumPosition;
}
//Collision check for cube and plane - taken from Tutorial Six
bool IsCollidingCheck(std::vector<collisionInfo> &civ, const cPlaneCollider &p, const cBoxCollider &b) {
  const dvec3 pp = p.GetParent()->GetPosition();
  const dvec3 bp = b.GetParent()->GetPosition();

  // local coords on cube
  dvec3 points[8] = {dvec3(b.radius, b.radius, b.radius),   dvec3(-b.radius, b.radius, b.radius),
                     dvec3(b.radius, -b.radius, b.radius),  dvec3(-b.radius, -b.radius, b.radius),
                     dvec3(b.radius, b.radius, -b.radius),  dvec3(-b.radius, b.radius, -b.radius),
                     dvec3(b.radius, -b.radius, -b.radius), dvec3(-b.radius, -b.radius, -b.radius)};

  // transfrom to global
  const mat4 m = glm::translate(bp) * mat4_cast(b.GetParent()->GetRotation());
  for (int i = 0; i < 8; i++) {
    points[i] = dvec3(m * dvec4(points[i], 1.0));
  }

  // For each point on the cube, which side of cube are they on?
  double distances[8];
  bool isCollided = false;
  for (int i = 0; i < 8; i++) {
    dvec3 planeNormal = p.normal;

    distances[i] = dot(pp, planeNormal) - dot(points[i], planeNormal);

    if (distances[i] > 0) {
      //	 cout << "CuboidPlane!\n";
      civ.push_back({&p, &b, points[i] + planeNormal * distances[i], planeNormal, distances[i]});
      isCollided = true;
    }
  }
  return isCollided;
}
// Box Collider
bool IsCollidingCheck(std::vector<collisionInfo> &civ, const cBoxCollider &c1, const cBoxCollider &c2) {
	// Get position for each cube
	const dvec3 c1P = c1.GetParent()->GetPosition();
	const dvec3 c2P = c2.GetParent()->GetPosition();
	// Calculate the radius' added together
	const double sumRadius = c1.radius + c2.radius;
	// Get the vector for the distance between both points
	const dvec3 d = c2P - c1P;
	// Get the length of the above vector
	const double distance = glm::length(d);

	//Get's the points of the corners
	dvec3 pointsC1[8] = {/*[0]front top right*/ dvec3(c1.radius, c1.radius, c1.radius), /*[1]front top left*/dvec3(-c1.radius, c1.radius, c1.radius),
		/*[2]front bottom right*/dvec3(c1.radius, -c1.radius, c1.radius), /*[3]front bottom left*/dvec3(-c1.radius, -c1.radius, c1.radius),
		/*[4] back top right */dvec3(c1.radius, c1.radius, -c1.radius), /*[5] back top left */dvec3(-c1.radius, c1.radius, -c1.radius),
		/*[6] back bottom right */dvec3(c1.radius, -c1.radius, -c1.radius), /*[7]back bottom left*/dvec3(-c1.radius, -c1.radius, -c1.radius) };
	dvec3 pointsC2[8] = { dvec3(c2.radius, c2.radius, c2.radius), dvec3(-c2.radius, c2.radius, c2.radius),
		dvec3(c2.radius, -c2.radius, c2.radius), dvec3(-c2.radius, -c2.radius, c2.radius),
		dvec3(c2.radius, c2.radius, -c2.radius), dvec3(-c2.radius, c2.radius, -c2.radius),
		dvec3(c2.radius, -c2.radius, -c2.radius), dvec3(-c2.radius, -c2.radius, -c2.radius) };
	// Transforms those points to global positions
	const mat4 mC1 = glm::translate(c1P) * mat4_cast(c1.GetParent()->GetRotation());
	const mat4 mC2 = glm::translate(c2P) * mat4_cast(c2.GetParent()->GetRotation());
	for (int i = 0; i < 8; i++) {
		pointsC1[i] = dvec3(mC1 * dvec4(pointsC1[i], 1.0));
		pointsC2[i] = dvec3(mC2 * dvec4(pointsC2[i], 1.0));
	}
	// Sets up the ranges for the Bounding Box
	double xMinimum = c2P.x + pointsC2[1].x;
	double xMaximum = c2P.x + pointsC2[0].x;
	double yMinimum = c2P.y + pointsC2[0].y;
	double yMaximum = c2P.y + pointsC2[2].y;
	double zMinimum = c2P.z + pointsC2[2].z;
	double zMaximum = c2P.z + pointsC2[6].z;
	// Ensures that the bounding boxes ranges are correctly the largest or smallest values of x, y or z
	for (int i = 0; i < 8; i++)
	{
		if (pointsC2[i].x < xMinimum)
			xMinimum = pointsC2[i].x;
		else if (pointsC2[i].x > xMaximum)
			xMaximum = pointsC2[i].x;
		if (pointsC2[i].y < yMinimum)
			yMinimum = pointsC2[i].y;
		else if (pointsC2[i].y > yMaximum)
			yMaximum = pointsC2[i].y;
		if (pointsC2[i].z < zMinimum)
			zMinimum = pointsC2[i].z;
		else if (pointsC2[i].z > zMaximum)
			zMaximum = pointsC2[i].z;
	}
	//Check if it's inside the sphere of the objects first
	//Multiplied by 0.6 to account for difference in radius
	if (distance < sumRadius * 0.6)
	{
		for (int i = 0; i < 8; i++) {
			//Checks each corner for a collision
			if (cornerCollision(pointsC1[i].x, xMinimum, xMaximum))
			{
				if (cornerCollision(pointsC1[i].y, yMinimum, yMaximum))
				{
					if (cornerCollision(pointsC1[i].z, zMinimum, zMaximum))
					{
						//Calculates the necessary values to pass into the collision resolver
						// Multiplied by 0.6 to account for difference in radius
						auto depth = sumRadius * 0.6 - distance;
						auto norm = glm::normalize(d);
						auto pos = c2P - depth * 0.5;
						civ.push_back({ &c1, &c2, pos, norm, depth*0.5});
						cout << "Collision!" << endl;
						return true;
					}
				}
			}
		}
	}
	return false;
}

bool IsColliding(std::vector<collisionInfo> &civ, const cCollider &c1, const cCollider &c2) {
  enum shape { UNKOWN = 0, PLANE, SPHERE, BOX };
  shape s1 = UNKOWN;
  shape s2 = UNKOWN;
  if (dynamic_cast<const cSphereCollider *>(&c1)) {
    s1 = SPHERE;
  } else if (dynamic_cast<const cPlaneCollider *>(&c1)) {
    s1 = PLANE;
  } else if (dynamic_cast<const cBoxCollider *>(&c1)) {
    s1 = BOX;
  }

  if (dynamic_cast<const cSphereCollider *>(&c2)) {
    s2 = SPHERE;
  } else if (dynamic_cast<const cPlaneCollider *>(&c2)) {
    s2 = PLANE;
  } else if (dynamic_cast<const cBoxCollider *>(&c2)) {
    s2 = BOX;
  }

  if (!s1 || !s2) {
    cout << "Routing Error" << endl;
    return false;
  }
  if (s1 == PLANE) {
    if (s2 == BOX) {
      return IsCollidingCheck(civ, dynamic_cast<const cPlaneCollider &>(c1), dynamic_cast<const cBoxCollider &>(c2));
    } else {
      cout << "Routing Error" << endl;
      return false;
    }
  } else if (s1 == BOX) {
    if (s2 == PLANE) {
      return IsCollidingCheck(civ, dynamic_cast<const cPlaneCollider &>(c2), dynamic_cast<const cBoxCollider &>(c1));
    } else if (s2 == BOX) {
      return IsCollidingCheck(civ, dynamic_cast<const cBoxCollider &>(c2), dynamic_cast<const cBoxCollider &>(c1));
    } else {
      cout << "Routing Error" << endl;
      return false;
    }
  }
  return false;
}
}