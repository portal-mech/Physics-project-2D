#pragma once
#include <glm/glm.hpp>
#include "Gizmos.h"
#include <vector>
#include <algorithm>

//this file is the header for the main physics scene and it's classes

class PhysicsObject;
class RigidBody;

const int NUM_MODES = 4; //number of draw modes

//types of draw modes
enum DrawMode {
	COLOUR,
	MASS,
	VELOCITY,
	ELASTICITY,
};

//main class of the physics engine, represents the enviroment in which the objects exist
class PhysicsScene
{
public:
	PhysicsScene();
	~PhysicsScene();

	void addActor(PhysicsObject* actor);	//add an actor to the scene
	void removeActor(PhysicsObject* actor); //remove an actor from the scene
	void update(float dt);					//main update
	void draw();							//main draw

	

	void setGravity(const glm::vec2 gravity) { m_gravity = gravity; }	//set gravity
	glm::vec2 getGravity() const { return m_gravity; }					//get gravity

	void setTimeStep(const float timeStep) { m_timeStep = timeStep; }	//set timestep
	float getTimeStep() const { return m_timeStep; }					//get timestep

	void setTimeScale(const float timeScale) { m_timeScale = timeScale; }	//set timescale
	float getTimeScale() const { return m_timeScale; }						//get timescale
	std::vector<PhysicsObject*> getActors() { return m_actors; }			//get actors

	void setDrawMode(DrawMode mode) { m_drawMode = mode; }	//set draw mode
	DrawMode getDrawMode() { return m_drawMode; }			//get draw mode

	//collision tests
	static bool plane2Plane(PhysicsObject*, PhysicsObject*);
	static bool plane2Sphere(PhysicsObject*, PhysicsObject*);
	static bool plane2Box(PhysicsObject*, PhysicsObject*);
	static bool sphere2Plane(PhysicsObject*, PhysicsObject*);
	static bool sphere2Sphere(PhysicsObject*, PhysicsObject*);
	static bool sphere2Box(PhysicsObject*, PhysicsObject*);
	static bool box2Plane(PhysicsObject*, PhysicsObject*);
	static bool box2Sphere(PhysicsObject*, PhysicsObject*);
	static bool box2Box(PhysicsObject*, PhysicsObject*);

	//apply contact forces between two objects
	static void ApplyContactForces(RigidBody* body1, RigidBody* body2, glm::vec2 norm, float pen);


protected:	
	glm::vec2 m_gravity;					//gravity
	float m_timeStep;						//simulation time step
	float m_timeScale;						//time scale modifier
	std::vector<PhysicsObject*> m_actors;	//list of all actors
	DrawMode m_drawMode;					//draw mode
};

//number of shape types
const int SHAPE_COUNT = 3;

//types of shapes
enum ShapeType {
	JOINT = -1,
	PLANE = 0,
	SPHERE,
	BOX,
};

//object within the physics scene
class PhysicsObject {
protected:
	PhysicsObject(ShapeType type) : ShapeID(type) {}

public:
	virtual void fixedUpdate(glm::vec2 gravity, float timestep) = 0;	//fixed timestep update
	virtual void draw(DrawMode mode) = 0;								//draw object
	virtual void resetPosition() {}										//reset object position

	ShapeType getShapeID() { return ShapeID; }							//get object shape
	glm::vec2 getPosition() { return m_position; }						//get object position
	void setPosition(glm::vec2 pos) { m_position = pos; }				//set object position

	bool getHighlight() { return m_highlight; }							//get object highlight
	void setHighlight(bool state) { m_highlight = state; }				//set object highlight

protected:
	ShapeType ShapeID;			//shape of object
	glm::vec2 m_position;		//position of object
	bool m_highlight = false;	//is the object highlighted
};

//minimum velocities
const float MIN_LINEAR_THRESHOLD = 0.05f;
const float MIN_ANGULAR_THRESHOLD = 0.05f;

//dynamic object within the ohysics scene
class RigidBody : public PhysicsObject {
public:
	RigidBody(ShapeType shapeID, glm::vec2 position,
		glm::vec2 velocity,float angularVelocity, float orientation, float mass, float elasticity);
	~RigidBody();

	virtual void fixedUpdate(glm::vec2 gravity, float timestep); //fixed timestep update
	void applyForce(glm::vec2 force, glm::vec2 pos); //apply a force to the rigidbody at a position
	void applyForce(glm::vec2 force); //apply a force to the rigidbody

	void resloveCollision(RigidBody* actor2, glm::vec2 contact, glm::vec2* collisionNormal = nullptr, float pen = 0); //resolve collision between two rigidbodys

	glm::vec2 toWorld(glm::vec2 point) { return m_position + m_localX * point.x + m_localY * point.y; }	//convert a local point to world position
	glm::vec2 toLocal(glm::vec2 point) { return  glm::vec2(glm::dot(m_localX, point - m_position), glm::dot(m_localY,point - m_position)); } //convert a world position to local point
	virtual bool isInside(glm::vec2 point) { return false; }	//checks if a point is inside the object


	float getOrientatation() { return m_orientation; }	//object orientation

	glm::vec2 getVelocity() { return m_velocity; }		//get and set velocity
	void setVelocity(glm::vec2 v) { m_velocity = v; }

	float getAngularVelocity() { return m_angularVelocity; }		//get and set angular velocity
	void setAngularVelocity(float v) { m_angularVelocity = v; }

	glm::vec2 getLocalX() { return m_localX; }	//get local x axis
	glm::vec2 getLocalY() { return m_localY; }	//get local y axis

	float getMass() { return m_isKinematic ? INT_MAX : m_mass; }		//get object mass
	float getMoment() { return m_isKinematic ? INT_MAX : m_moment; }	//get object moment
	float getElasticity() { return m_elasticity; }						//get object elasticity

	float getLinearDrag() { return m_linearDrag; }		//get linear drag
	float getAngularDrag() { return m_angularDrag; }	//get angular drag

	void setKinematic(bool state) { m_isKinematic = state; }	//get and set kinematic status
	bool isKinematic() { return m_isKinematic; }
	
protected:
	float m_orientation;	//object orientation

	glm::vec2 m_velocity;	//object velocity
	float m_angularVelocity;//object angular velocity

	glm::vec2 m_localX;		//object local x axis
	glm::vec2 m_localY;		//object local y axis
		
	float m_mass;			//object mass
	float m_moment;			//object moment
		
	float m_elasticity;		//object elasticity
	float m_linearDrag;		//object linear drag
	float m_angularDrag;	//object angular drag
	bool m_isKinematic;		//object kineamtic status
};

//infinte plane defined by its normal and distance from the origin
class Plane : public PhysicsObject {
public:
	Plane(glm::vec2 normal, float distance, glm::vec4 colour);
	~Plane();

	virtual void fixedUpdate(glm::vec2 gravity, float timeStep); //fixed timestep update
	virtual void draw(DrawMode mode);	//draw plane
	virtual void resetPosition();		//reset object position

	void resolveCollision(RigidBody* actor2, glm::vec2 contact);	//resolve colosion with a rigidbody

	glm::vec2 getNormal() { return m_normal; }			//get plane normal
	float getDistance() { return m_distanceToOrigin; }	//get plane distance to origin

protected:
	glm::vec2 m_normal;			//plane normal
	float m_distanceToOrigin;	//plane distance to origin
	glm::vec4 m_colour;			//plane colour
};

//dynamic sphere with collision detection
class Sphere : public RigidBody {
public:
	Sphere(glm::vec2 position, glm::vec2 velocity, float orientation, float mass, float elasticity, float radius, glm::vec4 colour);
	~Sphere();

	virtual void draw(DrawMode mode);	//draw sphere

	bool isInside(glm::vec2 point);		//is point inside the sphere

	float getRadius() { return m_radius; }		//get radius
	glm::vec4 getColour() { return m_colour; }	//get colour

protected:
	float m_radius;		//sphere radius
	glm::vec4 m_colour; //sphere colour
};

//oriented box with rotation and colision detection
class Box : public RigidBody {
public:
	Box(glm::vec2 position, glm::vec2 velocity, float width, float height, float orientation, float mass, float elasticity, glm::vec4 colour);
	~Box();

	virtual void draw(DrawMode mode);	//draw box

	bool checkBoxCorners(Box& box, glm::vec2& contact, int& numContacts, float& pen, glm::vec2& edgeNormal); //checks the box corners against another box
	bool isInside(glm::vec2 point);	//is point inside the box

	glm::vec2 getExtents() { return m_extents; }		//get box extents
	float getWidth() { return m_extents.x * 2.0f; }		//get box width
	float getHeight() { return m_extents.y * 2.0f; }	//get box height

protected:
	glm::vec2 m_extents;	//box extents
	glm::vec4 m_colour;		//box colour
};

//spring between two rigidbodys
class Spring : public PhysicsObject {
public:
	Spring(RigidBody* body1, RigidBody* body2, float restLength, float springCoefficient, float damping = 0.1f,
		glm::vec2 contact1 = glm::vec2(0, 0), glm::vec2 contact2 = glm::vec2(0, 0), glm::vec4 colour = glm::vec4(1,1,1,1));
	~Spring();

	void fixedUpdate(glm::vec2 gravity, float timestep); //fixed timestep update
	virtual void draw(DrawMode mode); //draw spring

	void setBody1(RigidBody* body) { m_body1 = body; }		//set rigidbodies
	void setBody2(RigidBody* body) { m_body2 = body; }		

	void setContact1(glm::vec2 pos) { m_contact1 = pos; }	//set contact points
	void setContact2(glm::vec2 pos) { m_contact2 = pos; }	

	RigidBody* getBody1() { return m_body1; }	//get rigidbodies
	RigidBody* getBody2() { return m_body2; }

	glm::vec2 getContact1();	//get contact points
	glm::vec2 getContact2();

	void setStrength(float strength) { m_springCoefficient = strength; } //set spring strength
	void resetRestLength();	//reset rest length

	bool lineCrosses(glm::vec2 point1, glm::vec2 point2); //test for intersection with a line segment

protected:
	RigidBody* m_body1;		//rigidbodies the spring connects
	RigidBody* m_body2;

	glm::vec2 m_contact1;   //contact points on the rigidbodies
	glm::vec2 m_contact2;

	float m_damping;			//damping amount
	float m_restLength;			//rest length
	float m_springCoefficient;	//spring strength

	glm::vec4 m_colour;			//spring colour
};