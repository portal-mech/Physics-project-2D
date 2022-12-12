#include "PhysicsScene.h"

//initalise main variables
PhysicsScene::PhysicsScene() {
	m_timeStep = 0.01f;
	m_timeScale = 1;
	m_gravity = { 0,0 };
	m_drawMode = COLOUR;
}

//dealocate memory
PhysicsScene::~PhysicsScene()
{
	for (auto pActor : m_actors) {
		delete pActor;
	}
}

//add an actor to the scene
void PhysicsScene::addActor(PhysicsObject* actor) {
	m_actors.push_back(actor);
}

//remove an actor from the scene
void PhysicsScene::removeActor(PhysicsObject* actor) {
	m_actors.erase(std::remove(m_actors.begin(), m_actors.end(), actor), m_actors.end());
}

//function pointer
typedef bool(*fn)(PhysicsObject*, PhysicsObject*);

//collision functions
static fn collisionFunctionArray[] = {
	PhysicsScene::plane2Plane,  PhysicsScene::plane2Sphere,  PhysicsScene::plane2Box,
	PhysicsScene::sphere2Plane, PhysicsScene::sphere2Sphere, PhysicsScene::sphere2Box,
	PhysicsScene::box2Plane,    PhysicsScene::box2Sphere,    PhysicsScene::box2Box,
};

//fixed time step update
void PhysicsScene::update(float dt) {

	static float accumulatedTime = 0.0f;
	accumulatedTime += dt;

	while (accumulatedTime >= m_timeStep) {
		for (auto pActor : m_actors) {
			pActor->fixedUpdate(m_gravity, m_timeStep * m_timeScale);
		}

		accumulatedTime -= m_timeStep;

		int actorCount = m_actors.size();
		//check for collision
		for (int outer = 0; outer < actorCount - 1; outer++) {
			for (int inner = outer + 1; inner < actorCount; inner++) {
				PhysicsObject* object1 = m_actors[outer];
				PhysicsObject* object2 = m_actors[inner];
				int shapeId1 = object1->getShapeID();
				int shapeId2 = object2->getShapeID();

				if (shapeId1 < 0 || shapeId2 < 0)
					continue;
				
				int functionIdx = (shapeId1 * SHAPE_COUNT) + shapeId2;
				fn collisionFunctionPtr = collisionFunctionArray[functionIdx];
				if (collisionFunctionPtr != nullptr) {
					collisionFunctionPtr(object1, object2);
				}
			}
		}
	}
}

//draw all objects
void PhysicsScene::draw() {
	for (auto pActor : m_actors) {
		pActor->draw(m_drawMode);
	}
}

//impossible as planes are static
bool PhysicsScene::plane2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return false;
}

//reverse arguments and call other function
bool PhysicsScene::plane2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return sphere2Plane(obj2,obj1);
}

//collision test between plane and box
bool PhysicsScene::plane2Box(PhysicsObject* obj1, PhysicsObject* obj2)
{
	Plane* plane = dynamic_cast<Plane*>(obj1);
	Box* box = dynamic_cast<Box*>(obj2);

	if (box != nullptr && plane != nullptr) {
		int numContacts = 0;
		glm::vec2 contact(0, 0);
		float contactV = 0;
		glm::vec2 planeOrigin = plane->getNormal() * plane->getDistance();

		//check box corners against plane
		for (float x = -box->getExtents().x; x < box->getWidth(); x += box->getWidth()) {
			for (float y = -box->getExtents().y; y < box->getHeight(); y += box->getHeight()) {
				//world space pos
				glm::vec2 p = box->getPosition() + x * box->getLocalX() + y * box->getLocalY();

				float distFromPlane = glm::dot(p - planeOrigin, plane->getNormal());

				glm::vec2 displacement = x * box->getLocalX() + y * box->getLocalY();
				glm::vec2 pointVelocity = box->getVelocity() + box->getAngularVelocity() * 
					glm::vec2(-displacement.y, displacement.x);

				float velocityIntoPlane = glm::dot(pointVelocity, plane->getNormal());

				if (distFromPlane < 0 && velocityIntoPlane <= 0) {
					numContacts++;
					contact += p;
					contactV += velocityIntoPlane;
				}
			}
		}
		if (numContacts > 0)
		{
			plane->resolveCollision(box, contact / (float)numContacts);
			return true;
		}
	}
	return false;
}

//collision test between sphere and plane
bool PhysicsScene::sphere2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	Sphere* sphere = dynamic_cast<Sphere*>(obj1);
	Plane* plane = dynamic_cast<Plane*>(obj2);

	if (sphere != nullptr && plane != nullptr) {
		glm::vec2 collisionNormal = plane->getNormal();
		//test distance to plane against sphere radius
		float sphereToPlane = glm::dot(sphere->getPosition(), plane->getNormal()) - plane->getDistance(); //distance of the centre of the sphere from the plane
		float intersection = sphere->getRadius() - sphereToPlane;

		float velocityOutOfPlane = glm::dot(sphere->getVelocity(), plane->getNormal());
		if (intersection > 0 && velocityOutOfPlane < 0) {
			glm::vec2 contact = sphere->getPosition() + (collisionNormal * -sphere->getRadius());
			plane->resolveCollision(sphere, contact);
			return true;
		}
	}
	return false;
}

//collision test between two spheres
bool PhysicsScene::sphere2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	Sphere* sphere1 = dynamic_cast<Sphere*>(obj1);
	Sphere* sphere2 = dynamic_cast<Sphere*>(obj2);

	if (sphere1 != nullptr && sphere2 != nullptr)
	{
		//compare radii of spheres to the distance between them
		float pen = sphere1->getRadius() + sphere2->getRadius() - glm::distance(sphere1->getPosition(),sphere2->getPosition());
		if (pen > 0) {
			sphere1->resloveCollision(sphere2, 0.5f * (sphere1->getPosition() +
				sphere2->getPosition()));
			return true;
		}
	}
	return false;
}

//reverse arguments and call other function
bool PhysicsScene::sphere2Box(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return box2Sphere(obj2,obj1);
}

//reverse arguments and call other function
bool PhysicsScene::box2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
		return plane2Box(obj2,obj1);
}

//collision test between box and sphere
bool PhysicsScene::box2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	Box* box = dynamic_cast<Box*>(obj1);
	Sphere* sphere = dynamic_cast<Sphere*>(obj2);

	if (box != nullptr && sphere != nullptr)
	{
		// transform the circle into the box's coordinate space 
		glm::vec2 circlePosWorld = sphere->getPosition() - box->getPosition();
		glm::vec2 circlePosBox = glm::vec2(glm::dot(circlePosWorld, box->getLocalX()), glm::dot(circlePosWorld, box->getLocalY()));

		// find the closest point to the circle centre on the box by clamping the coordinates in box - space to the box's extents 
		glm::vec2 closestPointOnBoxBox = circlePosBox;
		glm::vec2 extents = box->getExtents();
		if (closestPointOnBoxBox.x < -extents.x) closestPointOnBoxBox.x = -
			extents.x;
		if (closestPointOnBoxBox.x > extents.x) closestPointOnBoxBox.x = extents.x;
		if (closestPointOnBoxBox.y < -extents.y) closestPointOnBoxBox.y = -
			extents.y;
		if (closestPointOnBoxBox.y > extents.y) closestPointOnBoxBox.y = extents.y;
		// and convert back into world coordinates 
		glm::vec2 closestPointOnBoxWorld = box->getPosition() + closestPointOnBoxBox.x *
			box->getLocalX() + closestPointOnBoxBox.y * box->getLocalY();
		glm::vec2 circleToBox = sphere->getPosition() - closestPointOnBoxWorld;

		float penetration = sphere->getRadius() - glm::length(circleToBox);
		if (penetration > 0) {
			glm::vec2 direction = glm::normalize(circleToBox);
			glm::vec2 contact = closestPointOnBoxWorld;
			box->resloveCollision(sphere, contact, &direction, penetration);
			return true;
		}
	}
	return false;
}

//colision test between two boxes
bool PhysicsScene::box2Box(PhysicsObject* obj1, PhysicsObject* obj2)
{
	Box* box1 = dynamic_cast<Box*>(obj1);
	Box* box2 = dynamic_cast<Box*>(obj2);
	if (box1 != nullptr && box2 != nullptr) {
		glm::vec2 boxPos = box2->getPosition() - box1->getPosition();
		glm::vec2 norm(0, 0);
		glm::vec2 contact(0, 0);
		float pen = 0;
		int numContacts = 0;
		//test box corners against each other
		box1->checkBoxCorners(*box2, contact, numContacts, pen, norm);
		if (box2->checkBoxCorners(*box1, contact, numContacts, pen, norm)) {
			norm = -norm;
		}
		if (pen > 0) {
			box1->resloveCollision(box2, contact / float(numContacts), &norm, pen);
		}
		return true;
	}
	return false;
}

//used to separate objects that are colliding but not moving
void PhysicsScene::ApplyContactForces(RigidBody* body1, RigidBody* body2, glm::vec2 norm, float pen)
{
	float body2Mass = body2 ? body2->getMass() : INT_MAX;

	float body1Factor = body2Mass / (body1->getMass() + body2Mass);

	body1->setPosition(body1->getPosition() - body1Factor * norm * pen);
	if (body2)
		body2->setPosition(body2->getPosition() + (1 - body1Factor) * norm * pen);
}

//set base variables
RigidBody::RigidBody(ShapeType shapeID, glm::vec2 position,
	glm::vec2 velocity, float angularVelocity, float orientation, float mass, float elasticity) : PhysicsObject(shapeID)
{
	m_position = position;
	m_velocity = velocity;
	m_angularVelocity = angularVelocity;
	m_orientation = orientation;
	m_mass = mass;
	m_moment = 0;
	m_elasticity = elasticity;
	m_linearDrag = 0.2;
	m_angularDrag = 0.1f;
	m_isKinematic = false;
}

RigidBody::~RigidBody()
{
}

//apply a force to the object from a point
void RigidBody::applyForce(glm::vec2 force, glm::vec2 pos)
{
	m_velocity += force / getMass();
	m_angularVelocity += (force.y * pos.x - force.x * pos.y) / getMoment();
}

//apply a force to the object
void RigidBody::applyForce(glm::vec2 force)
{
	m_velocity += force / getMass();
}

//resolve a collision between two rigidbodies with a collision normal and a penatration depth
void RigidBody::resloveCollision(RigidBody* actor2, glm::vec2 contact, glm::vec2* collisionNormal, float pen)
{
	glm::vec2 normal = glm::normalize(collisionNormal ? *collisionNormal :
		actor2->m_position - m_position);

	glm::vec2 perp(normal.y, -normal.x);

	float r1 = glm::dot(contact - m_position, -perp);
	float r2 = glm::dot(contact - actor2->m_position, perp);

	float v1 = glm::dot(m_velocity, normal) - r1 * m_angularVelocity;
	float v2 = glm::dot(actor2->m_velocity, normal) - r2 * actor2->m_angularVelocity;

	if (v1 > v2) {
		float mass1 = 1.0f / (1.0f / m_mass + (r1 * r1) / m_moment);
		float mass2 = 1.0f / (1.0f / actor2->m_mass + (r2 * r2) / actor2->m_moment);

		float elasticity = (m_elasticity + actor2->m_elasticity) / 2.0f;

		glm::vec2 force = (1.0f + elasticity) * mass1 * mass2 / (mass1 + mass2) * (v1 - v2) * normal;

		applyForce(-force, contact - m_position);
		actor2->applyForce(force, contact - actor2->m_position);
		if (pen > 0)
			PhysicsScene::ApplyContactForces(this, actor2, normal, pen);
	}
	
}

//fixed timestep update
void RigidBody::fixedUpdate(glm::vec2 gravity, float timestep) {
	//set local axes
	float cs = cosf(m_orientation);
	float sn = sinf(m_orientation);
	m_localX = glm::normalize(glm::vec2(cs, sn));
	m_localY = glm::normalize(glm::vec2(-sn, cs));
	
	//ensure kinematic objects don't move
	if (m_isKinematic) {
		m_velocity = glm::vec2(0);
		m_angularVelocity = 0;
		return;
	}

	//apply drag
	m_velocity -= m_velocity * m_linearDrag * timestep;
	m_angularVelocity -= m_angularVelocity * m_angularDrag * timestep;
	
	//threshold values
	if (length(m_velocity) < MIN_LINEAR_THRESHOLD) {
		m_velocity = glm::vec2(0, 0);
	}
	if (abs(m_angularVelocity) < MIN_ANGULAR_THRESHOLD) {
		m_angularVelocity = 0;
	}

	//move objects
	m_position += m_velocity * timestep;
	applyForce(gravity * m_mass * timestep);

	m_orientation += m_angularVelocity * timestep;
}

//set base variables
Sphere::Sphere(glm::vec2 position, glm::vec2 velocity, float orientation, float mass, float elasticity, float radius, glm::vec4 colour) : RigidBody(SPHERE, position, velocity, 0, orientation, mass, elasticity)
{
	m_moment = 0.5f * mass * radius * radius;
	m_radius = radius;
	m_colour = colour;
}

//draw sphere
void Sphere::draw(DrawMode mode)
{
	glm::vec2 end = glm::vec2(std::cos(m_orientation), std::sin(m_orientation)) *
		m_radius;

	glm::vec4 colour = m_colour;
	//change colour based on draw mode
	switch (mode)
	{
	case COLOUR:
		break;
	case MASS:
		colour = m_isKinematic ? glm::vec4(1, 0, 0, 1) :
			glm::vec4(glm::max(m_mass * 0.025f, 0.05f), 0, 0, 1);
		break;
	case VELOCITY:
		colour = m_isKinematic ?  glm::vec4(0, 0, 0.1f, 1) :
			glm::vec4(0, 0, glm::max(glm::length(m_velocity) * 0.1f, 0.05f), 1);
		break;
	case ELASTICITY:
		colour = glm::vec4(0, glm::max(m_elasticity, 0.05f), 0, 1);
		break;
	default:
		break;
	}
	//highlight edge if selected
	if (m_highlight)
		aie::Gizmos::add2DCircle(m_position, m_radius * 1.05f + 0.25f, 20, glm::vec4(1,0.5,0,1));
	aie::Gizmos::add2DCircle(m_position, m_radius, 20, colour);
	aie::Gizmos::add2DLine(m_position, m_position + end, glm::vec4(1, 1, 1, 0.1));
}

//test if point is inside the sphere
bool Sphere::isInside(glm::vec2 point)
{
	if (glm::distance(point, m_position) < m_radius)
		return true;
	return false;
}

//set base propeties
Plane::Plane(glm::vec2 normal, float distance, glm::vec4 colour) : PhysicsObject(PLANE)
{
	m_normal = normal;
	m_distanceToOrigin = distance;
	m_position = m_normal * m_distanceToOrigin;
	m_colour = colour;
}

void Plane::fixedUpdate(glm::vec2 gravity, float timeStep) {}

//draw the plane
void Plane::draw(DrawMode mode)
{
	float lineSegmentLength = 300;
	glm::vec2 centerPoint = m_normal * m_distanceToOrigin;
	// easy to rotate normal through 90 degrees around z 
	glm::vec2 parallel(m_normal.y, -m_normal.x);
	glm::vec2 start = centerPoint + (parallel * lineSegmentLength);
	glm::vec2 end = centerPoint - (parallel * lineSegmentLength);
	glm::vec4 colour = m_colour;
	//change colour based on draw mode
	switch (mode)
	{
	case COLOUR:
		break;
	case MASS:
		colour = glm::vec4(1, 0, 0, 1);
		break;
	case VELOCITY:
		colour = glm::vec4(0, 0, 0.05, 1);
		break;
	case ELASTICITY:
		colour = glm::vec4(0, 0.5, 0, 1);
		break;
	default:
		break;
	}
	glm::vec4 colourFade = colour;
	colourFade.a = 0;
	//highlight segment if selected
	if (m_highlight)
		aie::Gizmos::add2DLine(start, end, glm::vec4(1, 0.5, 0, 1));
	aie::Gizmos::add2DTri(start, end, start - m_normal * 10.0f, colour, colour,
		colourFade);
	aie::Gizmos::add2DTri(end, end - m_normal * 10.0f, start - m_normal * 10.0f,
		colour, colourFade, colourFade);
}

void Plane::resetPosition() {}

//seperate collision resolution as planes are always static
void Plane::resolveCollision(RigidBody* actor2, glm::vec2 contact)
{
	glm::vec2 localContact = contact - actor2->getPosition();

	glm::vec2 vRel = actor2->getVelocity() + actor2->getAngularVelocity() * glm::vec2(-localContact.y, localContact.x);
	float velocityIntoPlane = glm::dot(vRel, m_normal);
	

	float e = actor2->getElasticity();

	float r = glm::dot(localContact, glm::vec2(m_normal.y, -m_normal.x));

	float mass0 = 1.0f / (1.0f / actor2->getMass() + (r * r) / actor2->getMoment());

	float j = -(1 + e) * velocityIntoPlane * mass0;

	glm::vec2 force = m_normal * j;

	float pen = glm::dot(contact, m_normal) - m_distanceToOrigin;
	PhysicsScene::ApplyContactForces(actor2, nullptr, m_normal, pen);

	actor2->applyForce(force, contact - actor2->getPosition());
}

//set base variables
Box::Box(glm::vec2 position, glm::vec2 velocity, float width, float height, float orientation, float mass, float elasticity, glm::vec4 colour) :
	RigidBody(BOX, position, velocity, 0, orientation, mass, elasticity)
{
	m_moment = 1.0f / 12.0f * mass * width * height;
	m_extents = { width * 0.5f, height * 0.5f };
	m_colour = colour;
	float cs = cosf(m_orientation);
	float sn = sinf(m_orientation);
	m_localX = glm::normalize(glm::vec2(cs, sn));
	m_localY = glm::normalize(glm::vec2(-sn, cs));
}


//draw the box
void Box::draw(DrawMode mode)
{
	glm::vec2 p1 = m_position - m_localX * m_extents.x - m_localY * m_extents.y;
	glm::vec2 p2 = m_position + m_localX * m_extents.x - m_localY * m_extents.y;
	glm::vec2 p3 = m_position - m_localX * m_extents.x + m_localY * m_extents.y;
	glm::vec2 p4 = m_position + m_localX * m_extents.x + m_localY * m_extents.y;

	glm::vec4 colour = m_colour;
	//change colour based on draw mode
	switch (mode)
	{
	case COLOUR:
		break;
	case MASS:
		colour = m_isKinematic ? glm::vec4(1, 0, 0, 1) :
			glm::vec4(glm::max(m_mass * 0.025f, 0.05f), 0, 0, 1);
		break;
	case VELOCITY:
		colour = m_isKinematic ? glm::vec4(0, 0, 0.05f, 1) :
			glm::vec4(0, 0, glm::max(glm::length(m_velocity) * 0.1f, 0.05f), 1);
		break;
	case ELASTICITY:
		colour = glm::vec4(0, glm::max(m_elasticity, 0.05f), 0, 1);
		break;
	default:
		break;
	}

	aie::Gizmos::add2DTri(p1, p2, p4, colour);
	aie::Gizmos::add2DTri(p1, p4, p3, colour);
	//highlight edges if selected
	if (m_highlight) {
		aie::Gizmos::add2DLine(p1, p2, glm::vec4(1, 0.5, 0, 1));
		aie::Gizmos::add2DLine(p1, p3, glm::vec4(1, 0.5, 0, 1));
		aie::Gizmos::add2DLine(p3, p4, glm::vec4(1, 0.5, 0, 1));
		aie::Gizmos::add2DLine(p2, p4, glm::vec4(1, 0.5, 0, 1));
	}
}

//transforms a box into local space and tests corners for intersection
bool Box::checkBoxCorners(Box& box, glm::vec2& contact, int& numContacts, float& pen, glm::vec2& edgeNormal)
{
	float minX, maxX, minY, maxY;
	float boxW = box.getExtents().x * 2;
	float boxH = box.getExtents().y * 2;
	int numLocalContacts = 0;
	glm::vec2 localContact(0, 0);
	bool first = true;

	for (float x = -box.getExtents().x; x < boxW; x += boxW)
	{
		for (float y = -box.getExtents().y; y < boxH; y += boxH)
		{

			glm::vec2 p = box.getPosition() + x * box.m_localX + y * box.m_localY;

			glm::vec2 p0(glm::dot(p - m_position, m_localX),
				glm::dot(p - m_position, m_localY));

			if (first || p0.x < minX) minX = p0.x;
			if (first || p0.x > maxX) maxX = p0.x;
			if (first || p0.y < minY) minY = p0.y;
			if (first || p0.y > maxY) maxY = p0.y;

			if (p0.x >= -m_extents.x && p0.x <= m_extents.x &&
				p0.y >= -m_extents.y && p0.y <= m_extents.y)
			{
				numLocalContacts++;
				localContact += p0;
			}
			first = false;
		}
	}

	if (maxX <= -m_extents.x || minX >= m_extents.x ||
		maxY <= -m_extents.y || minY >= m_extents.y)
		return false;


	if (numLocalContacts == 0)
		return false;

	bool res = false;
	contact += m_position + (localContact.x * m_localX + localContact.y * m_localY) /
		(float)numLocalContacts;
	numContacts++;

	float pen0 = m_extents.x - minX;
	if (pen0 > 0 && (pen0 < pen || pen == 0)) {
		edgeNormal = m_localX;
		pen = pen0;
		res = true;
	}
	pen0 = maxX + m_extents.x;
	if (pen0 > 0 && (pen0 < pen || pen == 0)) {
		edgeNormal = -m_localX;
		pen = pen0;
		res = true;
	}
	pen0 = m_extents.y - minY;
	if (pen0 > 0 && (pen0 < pen || pen == 0)) {
		edgeNormal = m_localY;
		pen = pen0;
		res = true;
	}
	pen0 = maxY + m_extents.y;
	if (pen0 > 0 && (pen0 < pen || pen == 0)) {
		edgeNormal = -m_localY;
		pen = pen0;
		res = true;
	}
	return res;
}

//tests if a point is inside the box
bool Box::isInside(glm::vec2 point)
{
	glm::vec2 localPoint = toLocal(point);
	if (localPoint.x > m_extents.x || localPoint.x < -m_extents.x ||
		localPoint.y > m_extents.y || localPoint.y < -m_extents.y)
		return false;
	return true;
}

//inintalise varables
Spring::Spring(RigidBody* body1, RigidBody* body2, float restLength, float springCoefficient, float damping,
	glm::vec2 contact1, glm::vec2 contact2, glm::vec4 colour) : PhysicsObject(JOINT)
{
	m_body1 = body1;
	m_body2 = body2;

	m_contact1 = contact1;
	m_contact2 = contact2;

	m_damping = damping;
	//if no rest length given set it to the distance between contact points
	m_restLength = restLength < 0 ? glm::distance(getContact1(),getContact2()) : restLength;
	m_springCoefficient = springCoefficient;

	m_colour = colour;
	m_highlight = false;
}

//fixed timestep update
void Spring::fixedUpdate(glm::vec2 gravity, float timestep)
{
	// get the world coordinates of the ends of the springs 
	glm::vec2 p1 = getContact1();
	glm::vec2 p2 = getContact2();

	if (p1 == p2)
		p2.x += 0.01;

	float length = glm::distance(p1, p2);
	glm::vec2 direction = glm::normalize(p2 - p1);

	// apply damping 
	glm::vec2 relativeVelocity = (m_body2 ? m_body2->getVelocity() : glm::vec2(0)) - (m_body1 ? m_body1->getVelocity() : glm::vec2(0));

	// F = -kX - bv 
	glm::vec2 force = direction * m_springCoefficient * (m_restLength - length) -
		m_damping * relativeVelocity;

	const float threshold = 5000.0f;
	float forceMag = glm::length(force);
	if (forceMag > threshold)
		force *= threshold / forceMag;

	if (m_body1)
	m_body1->applyForce(-force * timestep, p1 - m_body1->getPosition());
	if (m_body2)
	m_body2->applyForce(force * timestep, p2 - m_body2->getPosition());
}

//draw the spring
void Spring::draw(DrawMode mode)
{
	glm::vec4 colour = m_colour;
	//change colour based on draw mode
	switch (mode)
	{
	case COLOUR:
		break;
	case MASS:
		colour = glm::vec4(0.1f, 0, 0, 1);
		break;
	case VELOCITY:
		colour = glm::vec4(0, 0, 0.5f, 1);
		break;
	case ELASTICITY:
		colour = glm::vec4(0, glm::max(m_springCoefficient * 0.01f, 0.05f), 0, 1);
		break;
	default:
		break;
	}
	if (m_highlight) {
		colour = glm::vec4(1, 0.5, 0, 1);
	}

	aie::Gizmos::add2DLine(getContact1(), getContact2(), colour);
}

//get contact point 1 in world space
glm::vec2 Spring::getContact1()
{
	return m_body1 ? m_body1->toWorld(m_contact1) : m_contact1;
}
//get contact point 2 in world space
glm::vec2 Spring::getContact2()
{
	return m_body2 ? m_body2->toWorld(m_contact2) : m_contact2;
}

//reset the rest length to the current length
void Spring::resetRestLength()
{
	m_restLength = glm::distance(getContact1(), getContact2());
}

//test for intersection with a line segment
bool Spring::lineCrosses(glm::vec2 point1, glm::vec2 point2)
{
	if (glm::max(point1.x, point2.x) < glm::min(getContact1().x, getContact2().x)) {
		return false;
	}

	if (glm::min(point1.x, point2.x) > glm::max(getContact1().x, getContact2().x)) {
		return false;
	}

	if (glm::max(point1.y, point2.y) < glm::min(getContact1().y, getContact2().y)) {
		return false;
	}

	if (glm::min(point1.y, point2.y) > glm::max(getContact1().y, getContact2().y)) {
		return false;
	}

	float A1 = (point1.y - point2.y) / (point1.x - point2.x);
	float A2 = (getContact1().y - getContact2().y) / (getContact1().x - getContact2().x);

	if (A1 == A2) {
		return false;
	}

	float b1 = point1.y - A1 * point1.x;
	float b2 = getContact1().y - A2 * getContact1().x;

	float Xa = (b2 - b1) / (A1 - A2);

	if ((Xa < glm::max(glm::min(point1.x, point2.x), glm::min(getContact1().x, getContact2().x))) || (Xa > glm::min(glm::max(point1.x, point2.x), glm::max(getContact1().x, getContact2().x)))) {
		return false;
	}
	return true;
}
