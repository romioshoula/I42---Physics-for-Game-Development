//SolarSpaceSystem_source.cpp RamiWailShoula
//Function extentions from Header file (definitions) and Function Header file and main for a 2D space solar system
//box2d/SFML SolarSpaceSystem

//#includes

#include"SolarSpaceSystem_header.h" //Our (powerful) header file xdxd ^^
#include"SolarSpaceSystem_functions.h" //All function implementations in one header file

//Basic includes
//I know these are redundant (placed in header files) but just for safety run

#include<iostream>
#include <SFML/Graphics.hpp>
#include <array>
#include <iterator>
#include <box2d/box2d.h>

using namespace std;
using namespace sf;

//Functions should be here
//but I collected them all and placed them in a function header file (#clean code #organizational coding GG)

//driving function //main
int main()
{
	//Printing functions to user
	//cout << "";
	cout << "Ok Hello! Here is a 'simple' universe created in box2d/SFML\n";
	cout << "Here are the keys: \n";
	cout << "1: Click the mouse anywhere to add a planet in space! (Right click, Left click, & mouse button work)\n";
	cout << "2: While clicking, drag to add an initial force (vector) to the planet\n";
	cout << "3: Keys: Hold Left shift 'LShift' key and click on a planet to delete (one planet at a time)\n";
	cout << "4: Keys: Hold Left control 'LControl' key and click on window, move mouse to take control and move the window during runtime\n";
	cout << "5: Different planet sizes (radius represents mass) have different color representations (Blue, Yellow, Orange, Red) respective to increasing mass (in 10x increments)\n";
	cout << "6: Default mass of planet is 10 (10,000,000,000,000 billion kg or 0.03% of Mercury_smallest planet_size which is 0.055 x Earth size\n";
	cout << "7: Keys: You can Increase/Decrease Planet size with 'PageUp'||'Add' / 'PageDown'||'Subtract' keys, respectively (rightmost keys on most keyboards)\n";
	cout << "8: Mass / PlanetCount indicator are shown in the window in DejaVuSans font (add it as a header file .ttf)\n";
	cout << "9: Keys: Press 'P' key to toggle showing paths on the window\n";
	cout << "10: Keys: Bonus Function: press 'C' key to Cluster a lot of planets in a spherical physical formation 'Protoplanetary disk' at the centre of current window\n";
	cout << "11: Keys: Zoom In/Out in wondow using mouse wheel (Up/Down), Respectively\n";
	cout << "12: Window closes on 'Esc' key or press x on window\n";
	cout << "13: Kindly note that when 2 planets colide they join in one planet with the sum of the masses (indicated by changing color of planets)\n";
	cout << "14: Keys: press 'Delete' key to delete/erase all planets\n";
	cout << "\nPlease See the demo video for all functions";
	cout << "\nThank You! Enjoy Space!\n";


	//Open window 
	RenderWindow window(VideoMode(1024, 768), "SpaceSolarSystem using box2d/SFML");
	//Create universe
	Universe universe;
	//Connect our universe to our window via interface
	Interface interface(window, universe);
	// Logic variables
	Clock timer;
	float delta_t(0);
	while (window.isOpen())
	{
		Event evt; //Handling events
		while (window.pollEvent(evt))
		{
			interface.handle_event(evt);
		}
		delta_t = timer.restart().asSeconds(); //Restart the timer for handling events
		if (delta_t > MAX_DELTA_T)
		{
			delta_t = MAX_DELTA_T;
		}
		//Move our universe on delta time
		universe.move(delta_t); 
		//Clear our wonderful window and prepare for magic! (Physics is magical)
		window.clear();
		//Now let's do it, draw this wonderful universe
		universe.draw(window);
		//Drawing the interface (universe on the window via interface)
		interface.draw();
		//Display greatness
		window.display();
	}
	//return 0;
	return EXIT_SUCCESS; //succesful space modelling 
}