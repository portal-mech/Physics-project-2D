#pragma once

#include "Application.h"
#include "Renderer2D.h"
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include "PhysicsScene.h"

//this file is the header for the main aplication and gui logic

class Application2D;

//member function pointer
typedef void (Application2D::*func)(int);

//gui button class
class Button {
public:
	Button(glm::vec2 pos, glm::vec2 size, const char* sprite1, const char* sprite2, func f, int val);
	~Button();
	bool Trigger(glm::vec2 pos, Application2D& app); //when the button is pressed run the function pointer
	bool isInside(glm::vec2 pos);					 //check if a point is inside the button
	void Set();										 //set button state
	void Reset();									 //reset button state
	void Draw(aie::Renderer2D* renderer);			 //draw the button

	void setPos(glm::vec2 pos) { m_pos = pos; }		 //set the button position
protected:
	func m_func;				//function to execute when pressed
	int m_val;					//argument to run the function with
	glm::vec2 m_pos;			//button position
	glm::vec2 m_size;			//button size
	aie::Texture* m_sprite1;	//button normal sprite
	aie::Texture* m_sprite2;	//button pressed sprite
	bool isTriggered = false;	//is the button pressed
};

class Application2D : public aie::Application {
public:

	Application2D();
	virtual ~Application2D();

	virtual bool startup();					//start the app
	virtual void shutdown();				//close the app
		
	virtual void update(float deltaTime);	//main update
	virtual void draw();					//draw the screen

	glm::vec2 screenToWorld(glm::vec2);		//convert screen to world coordinates

	void setTool(int);						//set selected tool
	void setToolTip(int);					//show/hide tooltips
	void setDrawMode(int);					//set view
	void setPause(int);						//pause/unpause

protected:

	aie::Renderer2D*	m_2dRenderer;		//main renderer
	
	aie::Font*			m_font24;			//24pt font
	aie::Font*			m_font12;			//12pt font

	float m_timer;							//elapsed time

	float m_totalScroll;					//scroll wheel data
	float m_scrollLastFrame;				//
	float m_scrollDelta;					//

	const int NUM_TOOLS = 7;				//number of tools

	
	enum tool {								//tools that can be used
		SELECT,
		GRAB,
		SPHERE,
		BOX,
		PLANE,
		SPRING,
		TIME,
	};

	std::vector<Button*> buttons;			//gui buttons
	Button* m_toolTipButton;				//tooltip button
	Button* m_pauseButton;					//pause button
	bool m_CleanGUI = true;					//flag to clean the gui
	bool m_onGUI = false;					//if the mouse is on gui
	tool m_tool = SELECT;					//curent tool
	bool m_showToolTip = true;				//if tooltips should be shown

	RigidBody* m_grabObject = nullptr;		//grabbed object
	bool m_grabObjectKinematic = false;		//grabbed object's base kinematic state
	Spring* m_grabSpring = nullptr;			//spring for grab tool

	glm::vec2 m_mouseDownPos = glm::vec2(0);			//mouse click pos
	glm::vec4 m_objectColour = glm::vec4(1, 1, 1, 1);	//object to spawn colour
	glm::vec2 m_objectVelocity = { 0,0 };				//object to spawn velocity
	float m_objectAngularVelocity = 0;					//object to spawn angular velocity
	float m_objectOrientation = 0;						//object to spawn roatation
	float m_objectMass = 10;							//object to spawn mass
	float m_objectElasticity = 0.9f;					//object to spawn elasticity

	float m_objectRadius = 1;							//sphere to spawn radius

	float m_objectScaleX = 1;							//box to spawn x dimension
	float m_objectScaleY = 1;							//box to spawn y dimension

	glm::vec2 m_objectNrm = { 0, 1 };					//plane to spawn normal
	float m_objectDist = 0;								//plane to spawn distance

	Spring* m_toolSpring = nullptr;						//spring to spawn

	float m_timeModifier = 1;							//time scale modifier
	bool m_paused = false;								//pause state

	PhysicsScene* m_physicsScene;						//physics scene

};

