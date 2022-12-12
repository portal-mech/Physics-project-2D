#include "Application2D.h"
#include "Texture.h"
#include "Font.h"
#include "Input.h"
#include "Gizmos.h"
using namespace glm;

//this file contains the main aplication and gui logic

Application2D::Application2D() {

}

Application2D::~Application2D() {

}

bool Application2D::startup() {

	//set max tris
	aie::Gizmos::create(255U, 255U, 65535U, 65535U);

	m_2dRenderer = new aie::Renderer2D();

	//setup buttons
	buttons.push_back(new Button(vec2(40, getWindowHeight() - 74),vec2(64, 64), "./textures/toolicons/select1.png", "./textures/toolicons/select2.png", &Application2D::setTool, 0));
	buttons.push_back(new Button(vec2(40, getWindowHeight() - 138), vec2(64, 64), "./textures/toolicons/grab1.png", "./textures/toolicons/grab2.png", &Application2D::setTool, 1));
	buttons.push_back(new Button(vec2(40, getWindowHeight() - 202), vec2(64, 64), "./textures/toolicons/sphere1.png", "./textures/toolicons/sphere2.png", &Application2D::setTool, 2));
	buttons.push_back(new Button(vec2(40, getWindowHeight() - 266), vec2(64, 64), "./textures/toolicons/box1.png", "./textures/toolicons/box2.png", &Application2D::setTool, 3));
	buttons.push_back(new Button(vec2(40, getWindowHeight() - 330), vec2(64, 64), "./textures/toolicons/plane1.png", "./textures/toolicons/plane2.png", &Application2D::setTool, 4));
	buttons.push_back(new Button(vec2(40, getWindowHeight() - 394), vec2(64, 64), "./textures/toolicons/spring1.png", "./textures/toolicons/spring2.png", &Application2D::setTool, 5));
	buttons.push_back(new Button(vec2(40, getWindowHeight() - 458), vec2(64, 64), "./textures/toolicons/time1.png", "./textures/toolicons/time2.png", &Application2D::setTool, 6));
	buttons.push_back(new Button(vec2(getWindowWidth() - 232, 40), vec2(64, 64), "./textures/toolicons/colour1.png", "./textures/toolicons/colour2.png", &Application2D::setDrawMode, 0));
	buttons.push_back(new Button(vec2(getWindowWidth() - 168, 40), vec2(64, 64), "./textures/toolicons/mass1.png", "./textures/toolicons/mass2.png", &Application2D::setDrawMode, 1));
	buttons.push_back(new Button(vec2(getWindowWidth() - 104, 40), vec2(64, 64), "./textures/toolicons/velocity1.png", "./textures/toolicons/velocity2.png", &Application2D::setDrawMode, 2));
	buttons.push_back(new Button(vec2(getWindowWidth() - 40, 40), vec2(64, 64), "./textures/toolicons/elasticity1.png", "./textures/toolicons/elasticity2.png", &Application2D::setDrawMode, 3));
	m_toolTipButton = new Button(vec2(10, 10), vec2(32, 32), "./textures/toolicons/tooltip1.png", "./textures/toolicons/tooltip2.png", &Application2D::setToolTip, 0);
	buttons.push_back(m_toolTipButton);
	m_pauseButton = new Button(vec2(getWindowWidth() - 40, getWindowHeight() - 40), vec2(64, 64), "./textures/toolicons/pause1.png", "./textures/toolicons/pause2.png", &Application2D::setPause, 0);
	buttons.push_back(m_pauseButton);

	m_font24 = new aie::Font("./font/consolas.ttf", 24);
	m_font12 = new aie::Font("./font/consolas.ttf", 12);
	
	m_timer = 0;

	//initalise phsyics engine
	m_physicsScene = new PhysicsScene();
	m_physicsScene->setTimeStep(0.01f);

	m_physicsScene->setGravity(vec2(0, -9.82f));

	return true;
}

//dealocate resources
void Application2D::shutdown() {
	
	delete m_font12;
	delete m_font24;
	delete m_2dRenderer;
	delete m_physicsScene;
}

void Application2D::update(float deltaTime) {

	m_timer += deltaTime;

	aie::Gizmos::clear();

	//get input
	aie::Input* input = aie::Input::getInstance();

	//convert mouse pos to world pos
	vec2 pos = { input->getMouseX(), input->getMouseY() };
	pos = screenToWorld(pos);

	//get scroll since last frame
	m_totalScroll = input->getMouseScroll();
	m_scrollDelta = m_totalScroll - m_scrollLastFrame;

	//keybinds for tools
	if (input->wasKeyPressed(aie::INPUT_KEY_Q)) {
		m_tool = SELECT;
		m_CleanGUI = true;
	}
	if (input->wasKeyPressed(aie::INPUT_KEY_G)) {
		m_tool = GRAB;
		m_CleanGUI = true;
	}
	if (input->wasKeyPressed(aie::INPUT_KEY_S)) {
		m_tool = SPHERE;
		m_CleanGUI = true;
	}
	if (input->wasKeyPressed(aie::INPUT_KEY_B)) {
		m_tool = BOX;
		m_CleanGUI = true;
	}
	if (input->wasKeyPressed(aie::INPUT_KEY_P)) {
		m_tool = PLANE;
		m_CleanGUI = true;
	}
	if (input->wasKeyPressed(aie::INPUT_KEY_J)) {
		m_tool = SPRING;
		m_CleanGUI = true;
	}
	if (input->wasKeyPressed(aie::INPUT_KEY_T)) {
		m_tool = TIME;
		m_CleanGUI = true;
	}
	if (input->wasKeyPressed(aie::INPUT_KEY_C)) {
		m_physicsScene->setDrawMode(COLOUR);
		m_CleanGUI = true;
	}
	if (input->wasKeyPressed(aie::INPUT_KEY_M)) {
		m_physicsScene->setDrawMode(MASS);
		m_CleanGUI = true;
	}
	if (input->wasKeyPressed(aie::INPUT_KEY_V)) {
		m_physicsScene->setDrawMode(VELOCITY);
		m_CleanGUI = true;
	}
	if (input->wasKeyPressed(aie::INPUT_KEY_E)) {
		m_physicsScene->setDrawMode(ELASTICITY);
		m_CleanGUI = true;
	}

	//freeze objects
	if (input->wasKeyPressed(aie::INPUT_KEY_F)) {
		for (PhysicsObject* actor : m_physicsScene->getActors()) {
			RigidBody* obj = dynamic_cast<RigidBody*>(actor);
			if (obj) {
				obj->setVelocity(vec2(0));
			}
		}
	}

	//toggle kinematic
	if (input->wasKeyPressed(aie::INPUT_KEY_K)) {
		for (PhysicsObject* actor : m_physicsScene->getActors()) {
			RigidBody* obj = dynamic_cast<RigidBody*>(actor);
			if (obj) {
				obj->setKinematic(!obj->isKinematic());
			}
		}
	}

	//pause/unpause time
	if (input->wasKeyPressed(aie::INPUT_KEY_SPACE)) {
		if (m_paused) {
			m_physicsScene->setTimeScale(m_timeModifier);
		}
		else {
			m_timeModifier = m_physicsScene->getTimeScale();
			m_physicsScene->setTimeScale(0);
		}
		m_paused = !m_paused;
		m_CleanGUI = true;
	}

	//delete object
	if (input->wasKeyPressed(aie::INPUT_KEY_BACKSPACE) || input->wasKeyPressed(aie::INPUT_KEY_DELETE)) {
		std::vector<PhysicsObject*> deleteActors;
		for (PhysicsObject* actor : m_physicsScene->getActors()) {
			if (actor->getHighlight()) {
				deleteActors.push_back(actor);
			}
		}
		for (PhysicsObject* actor : deleteActors) {
			m_physicsScene->removeActor(actor);
		}
		deleteActors.clear();
	}

	//check if the mouse is over a button or the world
	m_onGUI = false;
	for (Button* button : buttons) {
		m_onGUI = m_onGUI || button->isInside({ input->getMouseX(), input->getMouseY() });
	}

	//left click functions
	if (input->wasMouseButtonPressed(aie::INPUT_MOUSE_BUTTON_LEFT)) {
		//press button
		if (m_onGUI) {
			for (Button* button : buttons) {
				m_onGUI = button->Trigger({ input->getMouseX(), input->getMouseY() }, (*this)) || m_onGUI;
			}
		}
		else {
			switch (m_tool)
			{
			//select objects
			case Application2D::SELECT:
				for (PhysicsObject* actor : m_physicsScene->getActors()) {
					if (actor) {
						actor->setHighlight(false);
					}
					RigidBody* obj = dynamic_cast<RigidBody*>(actor);
					if (obj && obj->isInside(pos)) {
						obj->setHighlight(true);
						break;
					}
				}
				m_mouseDownPos = pos;
				break;
			//grab objects
			case Application2D::GRAB:
				for (PhysicsObject* actor : m_physicsScene->getActors()) {
					RigidBody* obj = dynamic_cast<RigidBody*>(actor);
					if (obj && obj->isInside(pos)) {
						m_grabObject = obj;
						m_objectVelocity = obj->getVelocity();
						m_objectAngularVelocity = obj->getAngularVelocity();
						m_grabObjectKinematic = obj->isKinematic();
						break;
					}
				}
				break;
			//spawn spheres
			case Application2D::SPHERE:
				m_mouseDownPos = pos;
				break;
			//spawn boxes
			case Application2D::BOX:
				m_mouseDownPos = pos;
				break;
			//spawn planes
			case Application2D::PLANE:
				break;
			//create springs
			case Application2D::SPRING:
				for (PhysicsObject* actor : m_physicsScene->getActors()) {
					RigidBody* obj = dynamic_cast<RigidBody*>(actor);
					if (obj && obj->isInside(pos)) {
						m_toolSpring = new Spring(nullptr, obj, 0, 0, 0.5f, pos, obj->toLocal(pos));
						break;
					}
				}
				if (!m_toolSpring) {
					m_toolSpring = new Spring(nullptr, nullptr, 0, 0, 0.5f, pos, pos);
				}
				m_physicsScene->addActor(m_toolSpring);
				break;
			default:
				break;
			}
		}
			
	}
	if (!m_onGUI) {
		//left click hold
		if (input->isMouseButtonDown(aie::INPUT_MOUSE_BUTTON_LEFT)) {
			switch (m_tool)
			{
			//select objects
			case Application2D::SELECT:
				//click + drag selection
				if (input->isKeyDown(aie::INPUT_KEY_LEFT_SHIFT)) {
					vec2 min = glm::min(m_mouseDownPos, pos);
					vec2 max = glm::max(m_mouseDownPos, pos);
					vec2 p1 = vec2(min.x, max.y);
					vec2 p2 = vec2(max.x, min.y);
					aie::Gizmos::add2DLine(min, p1, vec4(1, 1, 0, 1));
					aie::Gizmos::add2DLine(min, p2, vec4(1, 1, 0, 1));
					aie::Gizmos::add2DLine(p1, max, vec4(1, 1, 0, 1));
					aie::Gizmos::add2DLine(p2, max, vec4(1, 1, 0, 1));

					for (PhysicsObject* actor : m_physicsScene->getActors()) {
						if (actor->getPosition().x > min.x&& actor->getPosition().y > min.y&&
							actor->getPosition().x < max.x && actor->getPosition().y < max.y) {
							actor->setHighlight(true);
						}
						else {
							actor->setHighlight(false);
						}
					}
				}
				break;
			//grab objects
			case Application2D::GRAB:
				if (m_grabObject) {
					m_grabObject->setPosition(pos);
					m_grabObject->setKinematic(true);
				}
				break;
			//spawn sphere
			case Application2D::SPHERE:
			{
				//show velocity preview
				vec2 endpoint = m_mouseDownPos - (pos - m_mouseDownPos);
				aie::Gizmos::add2DTri(endpoint,
					m_mouseDownPos - vec2((endpoint - m_mouseDownPos).y, -(endpoint - m_mouseDownPos).x) * 0.05f - (pos - m_mouseDownPos) * 0.5f,
					m_mouseDownPos - vec2(-(endpoint - m_mouseDownPos).y, (endpoint - m_mouseDownPos).x) * 0.05f - (pos - m_mouseDownPos) * 0.5f,
					vec4(0, 0, 1, 0.5), vec4(0, 0, 1, 0.2), vec4(0, 0, 1, 0.2));
				aie::Gizmos::add2DTri(pos,
					m_mouseDownPos - vec2(-(endpoint - m_mouseDownPos).y, (endpoint - m_mouseDownPos).x) * 0.025f - (pos - m_mouseDownPos) * 0.5f,
					m_mouseDownPos - vec2((endpoint - m_mouseDownPos).y, -(endpoint - m_mouseDownPos).x) * 0.025f - (pos - m_mouseDownPos) * 0.5f,
					vec4(1, 1, 1, 0.5), vec4(0, 0, 1, 0.2), vec4(0, 0, 1, 0.2));
				m_objectVelocity = m_mouseDownPos - pos;
			}
			break;
			//spawn box
			case Application2D::BOX:
			{
				//show velocity preview
				vec2 endpoint = m_mouseDownPos - (pos - m_mouseDownPos);
				aie::Gizmos::add2DTri(endpoint,
					m_mouseDownPos - vec2((endpoint - m_mouseDownPos).y, -(endpoint - m_mouseDownPos).x) * 0.05f - (pos - m_mouseDownPos) * 0.5f,
					m_mouseDownPos - vec2(-(endpoint - m_mouseDownPos).y, (endpoint - m_mouseDownPos).x) * 0.05f - (pos - m_mouseDownPos) * 0.5f,
					vec4(0, 0, 1, 0.5), vec4(0, 0, 1, 0.2), vec4(0, 0, 1, 0.2));
				aie::Gizmos::add2DTri(pos,
					m_mouseDownPos - vec2(-(endpoint - m_mouseDownPos).y, (endpoint - m_mouseDownPos).x) * 0.025f - (pos - m_mouseDownPos) * 0.5f,
					m_mouseDownPos - vec2((endpoint - m_mouseDownPos).y, -(endpoint - m_mouseDownPos).x) * 0.025f - (pos - m_mouseDownPos) * 0.5f,
					vec4(1, 1, 1, 0.5), vec4(0, 0, 1, 0.2), vec4(0, 0, 1, 0.2));
				m_objectVelocity = m_mouseDownPos - pos;
			}
			break;
			//spawn plane
			case Application2D::PLANE:
				break;
			//create spring
			case Application2D::SPRING:
				if (m_toolSpring) {
					m_toolSpring->setContact1(pos);
				}
				break;
			default:
				break;
			}

		}

		//left click mouse up
		if (input->wasMouseButtonReleased(aie::INPUT_MOUSE_BUTTON_LEFT)) {
			switch (m_tool)
			{
			case Application2D::GRAB:
				if (m_grabObject) {
					m_grabObject->setKinematic(m_grabObjectKinematic);
					m_grabObject->setVelocity(m_objectVelocity);
					m_grabObject->setAngularVelocity(m_objectAngularVelocity);
				}
				m_grabObject = nullptr;
				break;
			case Application2D::SPHERE:
			{
				Sphere* ball = new Sphere(m_mouseDownPos, m_objectVelocity, m_objectOrientation, m_objectMass, m_objectElasticity, m_objectRadius,
					m_objectColour);
				m_physicsScene->addActor(ball);
				m_objectColour = { (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, 1 };
				m_objectVelocity = vec2(0);
			}
			break;
			case Application2D::BOX:
			{
				Box* box = new Box(m_mouseDownPos, m_objectVelocity, m_objectScaleX, m_objectScaleY, m_objectOrientation, m_objectMass, m_objectElasticity, m_objectColour);
				m_physicsScene->addActor(box);
				m_objectColour = { (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, 1 };
				m_objectVelocity = vec2(0);
			}
			break;
			case Application2D::PLANE:
			{
				Plane* plane = new Plane(m_objectNrm, m_objectDist, m_objectColour);
				m_physicsScene->addActor(plane);
				m_objectColour = { (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, 1 };
			}
			break;
			case Application2D::SPRING:
				if (m_toolSpring) {
					for (PhysicsObject* actor : m_physicsScene->getActors()) {
						RigidBody* obj = dynamic_cast<RigidBody*>(actor);
						if (obj && obj != m_toolSpring->getBody2() && obj->isInside(pos)) {
							m_toolSpring->setBody1(obj);
							m_toolSpring->setContact1(obj->toLocal(pos));
							break;
						}
					}
					m_toolSpring->setStrength(1000);
					m_toolSpring->resetRestLength();
					m_toolSpring = nullptr;
				}
				break;
			}
		}

		//right click functions
		if (input->wasMouseButtonPressed(aie::INPUT_MOUSE_BUTTON_RIGHT)) {
			switch (m_tool)
			{
			//deselect objects
			case Application2D::SELECT:
				for (PhysicsObject* actor : m_physicsScene->getActors()) {
					Spring* obj = dynamic_cast<Spring*>(actor);
					if (obj) {
						obj->setHighlight(false);
					}
				}
				break;
			//grab objects with spring
			case Application2D::GRAB:
				for (PhysicsObject* actor : m_physicsScene->getActors()) {
					RigidBody* obj = dynamic_cast<RigidBody*>(actor);
					if (obj && obj->isInside(pos)) {
						m_grabSpring = new Spring(nullptr, obj, 0, 1000, 0.1f, pos, obj->toLocal(pos));
						m_physicsScene->addActor(m_grabSpring);
						break;
					}
				}
				break;
			//spawn kinematic sphere
			case Application2D::SPHERE:
			{
				Sphere* ball = new Sphere(pos, vec2(0), m_objectOrientation, m_objectMass, m_objectElasticity, m_objectRadius,
					{ (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, 1 });
				m_physicsScene->addActor(ball);
				ball->setKinematic(true);
				m_objectColour = { (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, 1 };
			}
			break;
			//spawn kinematic box
			case Application2D::BOX:
			{
				Box* box = new Box(m_mouseDownPos, m_objectVelocity, m_objectScaleX, m_objectScaleY, m_objectOrientation, m_objectMass, m_objectElasticity, m_objectColour);
				m_physicsScene->addActor(box);
				box->setKinematic(true);
				m_objectColour = { (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, 1 };
			}
			break;
			//select springs
			case Application2D::SPRING:
				for (PhysicsObject* actor : m_physicsScene->getActors()) {
					Spring* obj = dynamic_cast<Spring*>(actor);
					if (obj) {
						obj->setHighlight(false);
					}
				}
				m_mouseDownPos = pos;
				break;
			}

		}

		//right click hold
		if (input->isMouseButtonDown(aie::INPUT_MOUSE_BUTTON_RIGHT)) {
			switch (m_tool)
			{
			//grab objects with springs
			case Application2D::GRAB:
				if (m_grabSpring) {
					m_grabSpring->setContact1(pos);
				}
				break;
			//select springs
			case Application2D::SPRING:
				aie::Gizmos::add2DLine(m_mouseDownPos, pos, vec4(1, 1, 0, 0.9));
				break;
			default:
				break;
			}

		}

		//right click mouse up
		if (input->wasMouseButtonReleased(aie::INPUT_MOUSE_BUTTON_RIGHT)) {
			switch (m_tool)
			{
			//drop grabbed object
			case Application2D::GRAB:
				m_physicsScene->removeActor(m_grabSpring);
				m_grabSpring = nullptr;
				break;
			//select any springs that the drawn line intersects
			case Application2D::SPRING:
				for (PhysicsObject* actor : m_physicsScene->getActors()) {
					Spring* obj = dynamic_cast<Spring*>(actor);
					if (obj && obj->lineCrosses(m_mouseDownPos, pos)) {
						obj->setHighlight(true);
					}
				}
				break;
			//reset timescale
			case Application2D::TIME:
				m_timeModifier = 1;
				if (!m_paused)
				m_physicsScene->setTimeScale(m_timeModifier);
				break;
			default:
				break;
			}
		}

		//scrollwheel functions
		if (m_scrollDelta) {
			switch (m_tool)
			{
			//modify attributes of objects to be spawned
			case Application2D::SPHERE:
				//change mass
				if (input->isKeyDown(aie::INPUT_KEY_LEFT_SHIFT)) {
					m_objectMass += m_scrollDelta;
					if (m_objectMass < 1)
						m_objectMass = 1;
				}
				//change elasticity
				else if (input->isKeyDown(aie::INPUT_KEY_LEFT_CONTROL)) {
					m_objectElasticity += m_scrollDelta * 0.05;
					if (m_objectElasticity < 0)
						m_objectElasticity = 0;
					if (m_objectElasticity > 1)
						m_objectElasticity = 1;
				}
				//change rotation
				else if (input->isKeyDown(aie::INPUT_KEY_LEFT_ALT)) {
					m_objectOrientation += m_scrollDelta * 0.05;
					if (m_objectOrientation < 0)
						m_objectOrientation = 6.283f;
					if (m_objectOrientation > 6.283f)
						m_objectOrientation = 0;
				}
				//change scale
				else {
					m_objectRadius += m_scrollDelta * 0.5f;
					if (m_objectRadius < 0.5f)
						m_objectRadius = 0.5f;
				}
				break;
			case Application2D::BOX:
				//change mass
				if (input->isKeyDown(aie::INPUT_KEY_LEFT_SHIFT)) {
					m_objectMass += m_scrollDelta;
					if (m_objectMass < 1)
						m_objectMass = 1;
				}
				//change elasticity
				else if (input->isKeyDown(aie::INPUT_KEY_LEFT_CONTROL)) {
					m_objectElasticity += m_scrollDelta * 0.05;
					if (m_objectElasticity < 0)
						m_objectElasticity = 0;
					if (m_objectElasticity > 1)
						m_objectElasticity = 1;
				}
				//change rotation
				else if (input->isKeyDown(aie::INPUT_KEY_LEFT_ALT)) {
					m_objectOrientation += m_scrollDelta * 0.05;
					if (m_objectOrientation < 0)
						m_objectOrientation = 6.283f;
					if (m_objectOrientation > 6.283f)
						m_objectOrientation = 0;
				}
				//change scale X
				else if (input->isKeyDown(aie::INPUT_KEY_X)) {
					m_objectScaleX += m_scrollDelta * 0.5f;
					if (m_objectScaleX < 0.5f)
						m_objectScaleX = 0.5f;
				}
				//change scale Y
				else if (input->isKeyDown(aie::INPUT_KEY_Y)) {
					m_objectScaleY += m_scrollDelta * 0.5f;
					if (m_objectScaleY < 0.5f)
						m_objectScaleY = 0.5f;
				}
				//change global scale
				else {
					m_objectScaleX += m_scrollDelta * 0.5f;
					if (m_objectScaleX < 0.5f)
						m_objectScaleX = 0.5f;
					m_objectScaleY += m_scrollDelta * 0.5f;
					if (m_objectScaleY < 0.5f)
						m_objectScaleY = 0.5f;
				}
				break;
			case Application2D::PLANE:
			{
				//change rotation
				m_objectOrientation -= m_scrollDelta * 0.05236f;
				if (m_objectOrientation < 0)
					m_objectOrientation = 6.283f;
				if (m_objectOrientation > 6.283f)
					m_objectOrientation = 0;
				float cs = cosf(m_objectOrientation);
				float sn = sinf(m_objectOrientation);
				m_objectNrm = glm::normalize(glm::vec2(sn, cs));
			}
			break;
			//speed up/slow down time
			case Application2D::TIME:
				m_timeModifier += m_scrollDelta * 0.05f;
				if (m_timeModifier < 0)
					m_timeModifier = 0;
				if (m_timeModifier > 5)
					m_timeModifier = 5;
				if (!m_paused)
				m_physicsScene->setTimeScale(m_timeModifier);
				break;
			default:
				break;
			}
		}

		//draw previews for spawning tools
		switch (m_tool)
		{
			
		case Application2D::SPHERE:
		{
			if (input->isMouseButtonUp(aie::INPUT_MOUSE_BUTTON_LEFT))
				m_mouseDownPos = pos;

			glm::vec2 end = glm::vec2(std::cos(m_objectOrientation), std::sin(m_objectOrientation)) *
				m_objectRadius;
			glm::vec4 colour = m_objectColour * 0.5f;
			switch (m_physicsScene->getDrawMode())
			{
			case COLOUR:
				break;
			case MASS:
				colour = glm::vec4(glm::max(m_objectMass * 0.025f, 0.05f), 0, 0, 0.5f);
				break;
			case VELOCITY:
				colour = glm::vec4(0, 0, glm::max(glm::length(m_objectVelocity) * 0.1f, 0.1f), 0.5f);
				break;
			case ELASTICITY:
				colour = glm::vec4(0, glm::max(m_objectElasticity, 0.05f), 0, 0.5f);
				break;
			default:
				break;
			}
			aie::Gizmos::add2DCircle(m_mouseDownPos, m_objectRadius, 20, colour);
			aie::Gizmos::add2DLine(m_mouseDownPos, m_mouseDownPos + end, glm::vec4(1, 1, 1, 0.1));
		}
		break;
		case Application2D::BOX:
		{
			if (input->isMouseButtonUp(aie::INPUT_MOUSE_BUTTON_LEFT))
				m_mouseDownPos = pos;
			vec2 extents = { m_objectScaleX * 0.5f, m_objectScaleY * 0.5f };
			float cs = cosf(m_objectOrientation);
			float sn = sinf(m_objectOrientation);
			vec2 localX = glm::normalize(glm::vec2(cs, sn));
			vec2 localY = glm::normalize(glm::vec2(-sn, cs));

			glm::vec2 p1 = m_mouseDownPos - localX * extents.x - localY * extents.y;
			glm::vec2 p2 = m_mouseDownPos + localX * extents.x - localY * extents.y;
			glm::vec2 p3 = m_mouseDownPos - localX * extents.x + localY * extents.y;
			glm::vec2 p4 = m_mouseDownPos + localX * extents.x + localY * extents.y;

			glm::vec4 colour = m_objectColour;
			switch (m_physicsScene->getDrawMode())
			{
			case COLOUR:
				break;
			case MASS:
				colour = glm::vec4(glm::max(m_objectMass * 0.025f, 0.05f), 0, 0, 1);
				break;
			case VELOCITY:
				colour = glm::vec4(0, 0, glm::max(glm::length(m_objectVelocity) * 0.1f, 0.1f), 1);
				break;
			case ELASTICITY:
				colour = glm::vec4(0, glm::max(m_objectElasticity, 0.05f), 0, 1);
				break;
			default:
				break;
			}

			aie::Gizmos::add2DTri(p1, p2, p4, colour);
			aie::Gizmos::add2DTri(p1, p4, p3, colour);
		}
		break;
		case Application2D::PLANE:
		{
			if (input->isMouseButtonUp(aie::INPUT_MOUSE_BUTTON_LEFT))
				m_mouseDownPos = pos;

			m_objectDist = glm::dot(m_objectNrm, m_mouseDownPos);

			float lineSegmentLength = 300;
			glm::vec2 centerPoint = m_objectNrm * m_objectDist;
			// easy to rotate normal through 90 degrees around z 
			glm::vec2 parallel(m_objectNrm.y, -m_objectNrm.x);
			glm::vec2 start = centerPoint + (parallel * lineSegmentLength);
			glm::vec2 end = centerPoint - (parallel * lineSegmentLength);
			glm::vec4 colour = m_objectColour;
			switch (m_physicsScene->getDrawMode())
			{
			case COLOUR:
				break;
			case MASS:
				colour = glm::vec4(1, 0, 0, 1);
				break;
			case VELOCITY:
				colour = glm::vec4(0, 0, 0.1, 1);
				break;
			case ELASTICITY:
				colour = glm::vec4(0, 0.5, 0, 1);
				break;
			default:
				break;
			}
			glm::vec4 colourFade = colour;
			colourFade.a = 0;
			aie::Gizmos::add2DTri(start, end, start - m_objectNrm * 10.0f, colour, colour,
				colourFade);
			aie::Gizmos::add2DTri(end, end - m_objectNrm * 10.0f, start - m_objectNrm * 10.0f,
				colour, colourFade, colourFade);
		}
		break;
		case Application2D::SPRING:
			break;
		default:
			break;
		}

	}

	//redraw the gui
	if (m_CleanGUI) {
		for (Button* button : buttons) {
			button->Reset();
		}
		buttons[m_tool]->Set();
		buttons[NUM_TOOLS + m_physicsScene->getDrawMode()]->Set();
		if (m_paused)
			m_pauseButton->Set();
		if (m_showToolTip)
			m_toolTipButton->Set();
	}

	m_scrollLastFrame = m_totalScroll;

	//update physics
	m_physicsScene->update(deltaTime);
	//draw screen
	m_physicsScene->draw();

	// exit the application
	if (input->isKeyDown(aie::INPUT_KEY_ESCAPE))
		quit();
}

void Application2D::draw() {

	// wipe the screen to the background colour
	clearScreen();

	// begin drawing sprites
	m_2dRenderer->begin();
	static float aspectRatio = 16 / 9.f;
	aie::Gizmos::draw2D(glm::ortho<float>(-100, 100,
		-100 / aspectRatio, 100 / aspectRatio, -1.0f, 1.0f));
	
	char fps[32];
	sprintf_s(fps, 32, "FPS: %i", getFPS());
	m_2dRenderer->drawText(m_font24, fps, 0, getWindowHeight() - 24);
	m_2dRenderer->drawText(m_font12, "Press ESC to quit!", 0, getWindowHeight() - 36);
	//draw tooltips
	if (m_showToolTip) {
		switch (m_tool)
		{
		case Application2D::SELECT:
			m_toolTipButton->setPos(vec2(175, 105));
			m_2dRenderer->drawText(m_font24, "Select tool", 10, 100);
			m_2dRenderer->drawText(m_font12, "Left Click: Select object", 10, 85);
			m_2dRenderer->drawText(m_font12, "Shift + Left Click: Select Multiple", 10, 70);
			m_2dRenderer->drawText(m_font12, "F: Freeze velocity of selected", 10, 40);
			m_2dRenderer->drawText(m_font12, "K: Toggle kinematic on selected", 10, 25);
			m_2dRenderer->drawText(m_font12, "Del / Backspace: Delete selected", 10, 10);
			break;
		case Application2D::GRAB:
			m_toolTipButton->setPos(vec2(150, 55));
			m_2dRenderer->drawText(m_font24, "Grab tool", 10, 40);
			m_2dRenderer->drawText(m_font12, "Left Click: Grab object", 10, 25);
			m_2dRenderer->drawText(m_font12, "Right Click: Drag object", 10, 10);
			break;
		case Application2D::SPHERE:
			m_toolTipButton->setPos(vec2(175, 150));
			m_2dRenderer->drawText(m_font24, "Sphere tool", 10, 145);
			m_2dRenderer->drawText(m_font12, "Left Click: Create sphere", 10, 130);
			m_2dRenderer->drawText(m_font12, "Left Click + Drag: Create sphere with velocity", 10, 115);
			m_2dRenderer->drawText(m_font12, "Right Click: Create kinematic sphere", 10, 100);
			m_2dRenderer->drawText(m_font12, "Scroll: Change scale", 10, 70);
			m_2dRenderer->drawText(m_font12, "Shift + Scroll: Change mass", 10, 40);
			m_2dRenderer->drawText(m_font12, "Ctrl + Scroll: Change elasticity", 10, 25);
			m_2dRenderer->drawText(m_font12, "Alt + Scroll: Change orientation", 10, 10);
			break;
		case Application2D::BOX:
			m_toolTipButton->setPos(vec2(140, 180));
			m_2dRenderer->drawText(m_font24, "Box tool", 10, 175);
			m_2dRenderer->drawText(m_font12, "Left Click: Create box", 10, 160);
			m_2dRenderer->drawText(m_font12, "Left Click + Drag: Create box with velocity", 10, 145);
			m_2dRenderer->drawText(m_font12, "Right Click: Create kinematic sphere", 10, 130);
			m_2dRenderer->drawText(m_font12, "Scroll: Change scale", 10, 100);
			m_2dRenderer->drawText(m_font12, "Scroll + X: Change width", 10, 85);
			m_2dRenderer->drawText(m_font12, "Scroll + Y: Change height", 10, 70);
			m_2dRenderer->drawText(m_font12, "Shift + Scroll: Change mass", 10, 40);
			m_2dRenderer->drawText(m_font12, "Ctrl + Scroll: Change elasticity", 10, 25);
			m_2dRenderer->drawText(m_font12, "Alt + Scroll: Change orientation", 10, 10);
			break;
		case Application2D::PLANE:
			m_toolTipButton->setPos(vec2(160, 55));
			m_2dRenderer->drawText(m_font24, "Plane tool", 10, 40);
			m_2dRenderer->drawText(m_font12, "Left Click: Create plane", 10, 25);
			m_2dRenderer->drawText(m_font12, "Scroll: Change orientation", 10, 10);
			break;
		case Application2D::SPRING:
			m_toolTipButton->setPos(vec2(175, 55));
			m_2dRenderer->drawText(m_font24, "Spring tool", 10, 40);
			m_2dRenderer->drawText(m_font12, "Left Click + Drag: Create spring", 10, 25);
			m_2dRenderer->drawText(m_font12, "Right Click + Drag: Select springs", 10, 10);
			break;
		case Application2D::TIME:
			m_toolTipButton->setPos(vec2(150, 60));
			m_2dRenderer->drawText(m_font24, "Time tool", 10, 55);
			m_2dRenderer->drawText(m_font12, "Scroll: Change speed", 10, 40);
			m_2dRenderer->drawText(m_font12, "Space: Pause/unpause", 10, 25);
			m_2dRenderer->drawText(m_font12, "Right Click: Reset speed", 10, 10);

			break;
		default:
			break;
		}
		switch (m_physicsScene->getDrawMode())
		{
		case COLOUR:
			m_2dRenderer->drawText(m_font24, "Colour view", getWindowWidth() - 156.4, 85);
			break;
		case MASS:
			m_2dRenderer->drawText(m_font24, "Mass view", getWindowWidth() - 130, 85);
			break;
		case VELOCITY:
			m_2dRenderer->drawText(m_font24, "Velocity view", getWindowWidth() - 182.7, 85);
			break;
		case ELASTICITY:
			m_2dRenderer->drawText(m_font24, "Elasticity view", getWindowWidth() - 209.2, 85);
			break;
		default:
			break;
		}
		char timewarp[32];
		sprintf_s(timewarp, 32, "Speed: x%.2f", m_physicsScene->getTimeScale());
		m_2dRenderer->drawText(m_font24, timewarp, getWindowWidth() -235, getWindowHeight() - 24);
	}
	//draw tool names
	else {
		switch (m_tool)
		{
		case Application2D::SELECT:
			m_2dRenderer->drawText(m_font24, "Select tool", 10, 10);
			m_toolTipButton->setPos(vec2(175, 15));
			break;
		case Application2D::GRAB:
			m_2dRenderer->drawText(m_font24, "Grab tool", 10, 10);
			m_toolTipButton->setPos(vec2(150, 15));
			break;
		case Application2D::SPHERE:
			m_2dRenderer->drawText(m_font24, "Sphere tool", 10, 10);
			m_toolTipButton->setPos(vec2(175, 15));
			break;
		case Application2D::BOX:
			m_2dRenderer->drawText(m_font24, "Box tool", 10, 10);
			m_toolTipButton->setPos(vec2(140, 15));
			break;
		case Application2D::PLANE:
			m_2dRenderer->drawText(m_font24, "Plane tool", 10, 10);
			m_toolTipButton->setPos(vec2(160, 15));
			break;
		case Application2D::SPRING:
			m_2dRenderer->drawText(m_font24, "Spring tool", 10, 10);
			m_toolTipButton->setPos(vec2(175, 15));
			break;
		case Application2D::TIME:
			m_2dRenderer->drawText(m_font24, "Time tool", 10, 10);
			m_toolTipButton->setPos(vec2(150, 15));
			char timewarp[32];
			sprintf_s(timewarp, 32, "Speed: x%.2f", m_physicsScene->getTimeScale());
			m_2dRenderer->drawText(m_font24, timewarp, getWindowWidth() - 235, getWindowHeight() - 24);
			break;
		default:
			break;
		}
	}

	//draw buttons
	for (Button* button : buttons) {
		button->Draw(m_2dRenderer);
	}

	// done drawing sprites
	m_2dRenderer->end();
}

//convert a screen point to world coordinates
glm::vec2 Application2D::screenToWorld(glm::vec2 screenPos)
{
	vec2 worldPos = screenPos;

	worldPos.x -= getWindowWidth() / 2;
	worldPos.y -= getWindowHeight() / 2;

	worldPos.x *= 2.0f * 100 / getWindowWidth();
	worldPos.y *= 2.0f * 100 / ((16 / 9.f) * getWindowHeight());

	return worldPos;
}

//set the current tool
void Application2D::setTool(int tool)
{
	switch (tool)
	{
	case 0:
		m_tool = SELECT;
		break;
	case 1:
		m_tool = GRAB;
		break;
	case 2:
		m_tool = SPHERE;
		break;
	case 3:
		m_tool = BOX;
		break;
	case 4:
		m_tool = PLANE;
		break;
	case 5:
		m_tool = SPRING;
		break;
	case 6:
		m_tool = TIME;
		break;
	default:
		break;
	}
}

//set wheather tooltips should be shown
void Application2D::setToolTip(int)
{
	m_showToolTip = !m_showToolTip;
}

//set which propeties to display
void Application2D::setDrawMode(int mode)
{
	switch (mode)
	{
	case 0:
		m_physicsScene->setDrawMode(COLOUR);
		break;
	case 1:
		m_physicsScene->setDrawMode(MASS);
		break;
	case 2:
		m_physicsScene->setDrawMode(VELOCITY);
		break;
	case 3:
		m_physicsScene->setDrawMode(ELASTICITY);
		break;
	default:
		break;
	}
}

//set the pause state of the simulation
void Application2D::setPause(int)
{
	if (m_paused) {
		m_physicsScene->setTimeScale(m_timeModifier);
	}
	else {
		m_timeModifier = m_physicsScene->getTimeScale();
		m_physicsScene->setTimeScale(0);
	}
	m_paused = !m_paused;
}




//button constructor
Button::Button(glm::vec2 pos, glm::vec2 size, const char* sprite1, const char* sprite2, func f, int val)
{
	m_pos = pos;
	m_size = size;
	m_sprite1 = new aie::Texture(sprite1);
	m_sprite2 = new aie::Texture(sprite2);
	m_func = f;
	m_val = val;
}

//button trigger event
bool Button::Trigger(glm::vec2 pos, Application2D& app)
{
	if (pos.x < m_pos.x  - m_size.x * 0.5 || pos.y < m_pos.y - m_size.y * 0.5 ||
		pos.x > m_pos.x + m_size.x * 0.5 || pos.y > m_pos.y + m_size.y * 0.5) {
		return false;
	}
	(app.*m_func)(m_val);
	isTriggered = true;
	return true;
}

//check if point is inside the button
bool Button::isInside(glm::vec2 pos)
{
	if (pos.x < m_pos.x - m_size.x * 0.5 || pos.y < m_pos.y - m_size.y * 0.5 ||
		pos.x > m_pos.x + m_size.x * 0.5 || pos.y > m_pos.y + m_size.y * 0.5) {
		return false;
	}
	return true;
}

//set button trigger
void Button::Set()
{
	isTriggered = true;
}

//reset button trigger
void Button::Reset()
{
	isTriggered = false;
}

//draw the button
void Button::Draw(aie::Renderer2D* renderer)
{
	if (!isTriggered) {
		renderer->drawSprite(m_sprite1, m_pos.x, m_pos.y, m_size.x, m_size.y);
	}
	else {
		renderer->drawSprite(m_sprite2, m_pos.x, m_pos.y, m_size.x, m_size.y);
	}
}
