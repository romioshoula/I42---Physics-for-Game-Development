#pragma once
//SolarSpaceSystem_header.h RamiWailShoula //One header to bind them all xd (LOTR reference)
//Entire Header for box2d/SFML SolarSpaceSystem

//Definistion of header

#ifndef SOLARSPACESYSTEM_HPP
#define SOLARSPACESYSTEM_HPP
//#includes

#include <SFML/Graphics.hpp>
#include <iostream>
#include <random>
#include <functional>
#include <cmath>
#include <vector>
#include <list>
#include <iterator>
#include <array>

//using namepaces
using namespace sf;
using namespace std;

//Global definitions

#define MAX_DELTA_T (1.0f/30) //Delta Timer
#ifndef M_PI
#define M_PI 3.14 //Pie Pi xd
#endif
//Mass and clustering definitions
#define MASS 10
#define BASE_LINE 2
#define CLUSTER_RADIUS 100
#define CLUSTER_NUMBER 1000
#define CLUSTER_MASS 1000
#define G 1			//Newton's Gravitational constant //Big G //F=G*(m1*m2)/(r^2)
					//G=6.67430(15)*10^(-11) m^3.kg^(–1).s^(–2)
					//Used G=1 for simplicity only (simple conversions)
#define DENSITY 100 // kg/m^3

//Classes
//1
class Vector2d //2D vector (x,y vectors) class
{
public:
	float x;
	float y;

	Vector2d();
	Vector2d(float x, float y);
	Vector2d operator+(Vector2d other);
	Vector2d operator-(Vector2d other);
	Vector2d operator+=(Vector2d other);
	Vector2d operator-=(Vector2d other);
	Vector2d operator=(Vector2d other);
	bool operator==(Vector2d other);
	Vector2d operator*(float a);
	Vector2d operator*=(float a);
	friend Vector2d operator*(float a, Vector2d vector);
	friend Vector2d operator*=(float a, Vector2d vector);
	float Dot(Vector2d other);
	float getMagnitude();
	Vector2d getNormalized();
	void normalize();
	operator Vector2f();
	Vector2d getNormal();

	void print();
};
//2
class SATCollider //Planet collider class using 2D vectors
{
	Vector2d position;
public:
	vector<Vector2d> points;
	void updatePosition(Vector2d position);

	bool checkCollision(SATCollider other);
	vector<Vector2d> getNormals();
	Vector2d projectShape(Vector2d axis);
};
//3
class Renderer  //Renderer class for saving 
{
private:
	list<Drawable*> drawables;
public:
	void AddDrawable(Drawable* drawable);
	void Render(RenderWindow* window);
};
//4
class AABBCollider //Colider class using axis-aligned bounding boxes (AABB) algorithm
{					//Quickest algorithm to determine whether the two game entities are overlapping or not
public:
	// upper left 
	Vector2d ul;

	//lower right 
	Vector2d lr;
	Vector2d position;

	void updatePosition(Vector2d position);

	bool checkCollision(AABBCollider other);
};
//5
class Collider //Collider class to handle collision checking
{
public:
	float r;
	Vector2d center;

	bool checkCollision(Collider other)
	{
		//cout << (other.center - center).getMagnitude()<<" " << (other.r + r)<< endl;
		return (other.center - center).getMagnitude() < (other.r + r);
	}
};
//6
class Particle
{
public:
	Vector2d postion;
	Vector2d velocity;
	Vector2d acceleration;
	Vector2d forces;

	Collider collider;
	AABBCollider collider_aabb;
	SATCollider collider_sat;

	float mass;
	float drag;



	Particle(Vector2d postion, float mass = 1, float drag = 0.3f);
	void Update(float dt);
	void addForce(Vector2d force);
};
//7
class PhysicsWorld  //CLass to handle particle physics in 2 dimensions
{
private:
	list<Particle*> particles;
public:
	void addParticle(Particle* particle);
	void Update(float deltaTime);

	void checkCollision();
	void checktwoCircleCollision();
	void checkAABBCollision();
	void checkSatCollision();

	static const Vector2d gravity;
};
//8
class Body //CLass to handle circle masses bodies
{
public:
	Body(sf::Vector2f pos, uint64_t m, sf::Vector2f dir, bool with_path);

	void move(float dt);
	void applyGravityOf(const Body& b, float dt);
	void draw(sf::RenderWindow& window);
	bool collideWith(const Body& b) const;
	bool contains(const sf::Vector2f& point) const;
	void setPathEnabled(bool state);
	sf::VertexArray getPath() const;

	static float radiusForMass(uint64_t mass);

	sf::Vector2f position;
	sf::Vector2f direction;
	uint64_t mass;
	float radius;
	bool show_path;

private:
	float getDistanceTo(const Body& b) const;
	sf::Vector2f getCenter() const;

	sf::CircleShape shape;
	sf::VertexArray path;
};

// Handy helper functions

// ratio is a number between 0.0 and 1.0
template <typename T, typename Real>
T interpolate(T a, T b, Real ratio)
{
	return (b - a) * ratio + a;
}

template <typename Real>
sf::Color interpolate(sf::Color a, sf::Color b, Real ratio)
{
	sf::Color c;
	c.r = (b.r - a.r) * ratio + a.r;
	c.g = (b.g - a.g) * ratio + a.g;
	c.b = (b.b - a.b) * ratio + a.b;

	return c;
}
//9
class Universe	//Universal class to handle all Global Planet functions
{
public:
	Universe();

	void addPlanet(sf::Vector2f position, uint64_t m, sf::Vector2f dir);
	void addPlanet(Body p);
	void createProtodisk(const int number, const int radius, const int mass, const sf::Vector2f& position);
	void togglePath();

	uint64_t getPlanetNumber() const;
	bool isPathEnabled() const;

	void move(float delta_t);
	void draw(sf::RenderWindow& window);
	void eraseAt(const sf::Vector2f& pos);
	void eraseAll();

private:
	void savePlanetPath(const Body& b);

	std::vector<Body> planets; //Vector to save and handle planets data
	std::vector<sf::VertexArray> old_paths;
	bool show_path;

	// Protodisk stuff
	std::mt19937 rng;
};

Body combinedPlanets(const Body& a, const Body& b);
sf::Vector2f tangentThroughPoint(sf::Vector2f circle_center, sf::Vector2f point);

//10 
class Interface {	//Class to handle window operations (keyboard key functions)

public:
	Interface(sf::RenderWindow& win, Universe& verse);

	void handle_event(sf::Event evt);
	void draw();

private:
	struct {
		bool is_placing;
		bool is_moving;
		bool control;
		bool shift;
	} modifiers;

	Universe& universe;
	sf::RenderWindow& window;

	sf::View view;
	sf::CircleShape base_line;
	sf::VertexArray line;
	sf::Font font;
	sf::Text mass_text;
	sf::Text num_planets_text;
	sf::Text paths_text;

	uint64_t mass;
	uint64_t num_planets;
	sf::Vector2f mousePosition;
	sf::Vector2f start;
	sf::Vector2f end;
};

#endif // SOLARSPACESYSTEM_HPP