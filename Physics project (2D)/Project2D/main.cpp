#include "Application2D.h"

//main function for the program
int main() {
	
	// allocation
	auto app = new Application2D();

	// initialise and loop
	app->run("Physics Demo", 1280, 720, false);

	// deallocation
	delete app;

	return 0;
}