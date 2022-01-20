#pragma once
//SolarSpaceSystem_functions.h RamiWailShoula //One header file to bind them all xd (LOTR reference), for functions
//Entire Header for box2d/SFML SolarSpaceSystem

//Definistion of header

#ifndef SOLARSPACESYSTEM_FUNCTIONS_HPP
#define SOLARSPACESYSTEM_FUNCTIONS_HPP
//#includes

#include"SolarSpaceSystem_header.h" //Our (powerful) header file xdxd ^^

//Basic includes

#include<iostream>
#include <SFML/Graphics.hpp>
#include <array>
#include <iterator>
#include <box2d/box2d.h>

using namespace std;
using namespace sf;

//Functions

//1//Vector2d Functions
Vector2d::Vector2d()
{
	x = y = 0;
}
Vector2d::Vector2d(float x, float y)
{
	this->x = x;
	this->y = y;
}
Vector2d Vector2d::operator+(Vector2d other)
{
	return Vector2d(x + other.x, y + other.y);
}
Vector2d Vector2d::operator-(Vector2d other)
{
	return Vector2d(x - other.x, y - other.y);
}
Vector2d Vector2d::operator+=(Vector2d other)
{
	return Vector2d(x += other.x, y += other.y);
}
Vector2d Vector2d::operator-=(Vector2d other)
{
	return Vector2d(x -= other.x, y -= other.y);
}
Vector2d Vector2d::operator=(Vector2d other)
{
	return Vector2d(x = other.x, y = other.y);
}
bool Vector2d::operator==(Vector2d other)
{
	return x == other.x && y == other.y;
}
Vector2d Vector2d::operator*(float a)
{
	return Vector2d(a * x, a * y);
}
Vector2d Vector2d::operator*=(float a)
{
	return Vector2d(x *= a, y *= a);
}
float Vector2d::Dot(Vector2d other)
{
	return x * other.x + y * other.y;
}
float Vector2d::getMagnitude()
{
	return sqrt(x * x + y * y);
}
Vector2d Vector2d::getNormalized()
{
	float mag = getMagnitude();
	return Vector2d(x / mag, y / mag);
}
void Vector2d::normalize()
{
	float mag = getMagnitude();
	x /= mag; y /= mag;
}
Vector2d::operator Vector2f()
{
	return Vector2f(x, y);
}
Vector2d Vector2d::getNormal()
{
	return Vector2d(-y, x);
}
void Vector2d::print()
{
	cout << "<" << x << " , " << y << ">" << endl;
}
Vector2d operator*(float a, Vector2d vector)
{
	return vector * a;
}
Vector2d operator*=(float a, Vector2d vector)
{
	return vector *= a;
}
//end Vector 2d Functions

//2//SATCollider Functions
void SATCollider::updatePosition(Vector2d position)
{
	Vector2d delta = this->position - position;
	for (size_t i = 0; i < points.size(); i++)
	{
		points[i] += delta;
	}
	this->position = position;
}
bool SATCollider::checkCollision(SATCollider other)
{
	vector<Vector2d> normals = getNormals();
	for (size_t i = 0; i < normals.size(); i++)
	{
		Vector2d otherShape = other.projectShape(normals[i]);
		Vector2d shape = projectShape(normals[i]);

		if (shape.y < otherShape.x) return false;
		if (shape.x > otherShape.y) return false;
	}
	normals = other.getNormals();
	for (size_t i = 0; i < normals.size(); i++)
	{
		Vector2d otherShape = other.projectShape(normals[i]);
		Vector2d shape = projectShape(normals[i]);

		if (shape.y < otherShape.x) return false;
		if (shape.x > otherShape.y) return false;
	}

	return true;
}
vector<Vector2d> SATCollider::getNormals()
{
	vector<Vector2d> normals;
	for (size_t i = 0; i < points.size() - 1; i++)
	{
		Vector2d side = points[i + 1] - points[i];
		normals.push_back(side.getNormal().getNormalized());
	}

	Vector2d side = points[0] - points[points.size() - 1];
	normals.push_back(side.getNormal().getNormalized());

	return normals;
}
Vector2d SATCollider::projectShape(Vector2d axis)
{
	float dot = points[0].Dot(axis);
	float min = dot;
	float max = dot;
	for (size_t i = 1; i < points.size(); i++)
	{
		dot = points[i].Dot(axis);
		if (dot < min)min = dot;
		if (dot > max) max = dot;
	}
	return Vector2d(min, max);
}
//end SATCollider Functions

//3//Renderer Functions
void Renderer::AddDrawable(Drawable* drawable)
{
	drawables.push_back(drawable);
}
void Renderer::Render(RenderWindow* window)
{
	list<Drawable*>::iterator it;
	window->clear();
	for (it = drawables.begin(); it != drawables.end(); it++)
	{
		window->draw(**it);
	}
	window->display();
}
//end Renderer Functions

//4//AABBCollider Functions
void AABBCollider::updatePosition(Vector2d position)
{
	Vector2d delta = position - this->position;
	ul += delta;
	lr += delta;
	this->position = position;
}
bool AABBCollider::checkCollision(AABBCollider other)
{
	/*FloatRect rect(ul.x, ul.y, 40, 40);
	FloatRect rect2(other.ul.x, other.ul.y, 40, 40);
	return rect.intersects(rect2);*/
	// ul.x minX
	// lr.x maxX
	// ul.y minY
	// lr.y maxY
	if (ul.x >= other.ul.x && ul.x <= other.lr.x && ul.y >= other.ul.y && ul.y <= other.lr.y) return true;
	if (lr.x >= other.ul.x && lr.x <= other.lr.x && lr.y >= other.ul.y && lr.y <= other.lr.y) return true;
	return false;
}
//end AABBCollider Functions

//5//Collider Functions

//end Collider Functions

//6//Particle Functions
Particle::Particle(Vector2d postion, float mass, float drag)
{
	this->mass = mass;
	this->postion = postion;
	this->drag = drag;
	vector<Vector2d> points;
	points.push_back(Vector2d(0, 0));
	points.push_back(Vector2d(0, 40));
	points.push_back(Vector2d(40, 40));
	points.push_back(Vector2d(40, 0));
	collider_sat.points = points;
	collider_sat.updatePosition(postion);
	collider_aabb.ul = Vector2d(0, 0);
	collider_aabb.lr = Vector2d(40, 40);
	collider_aabb.updatePosition(postion);
	collider.center = postion;
	collider.r = 20;
}
void Particle::Update(float dt)
{
	if (velocity.getMagnitude() > 0)
	{
		Vector2d dragForce = /*-0.5f **/ -drag /** velocity.getMagnitude()*/ * velocity;
		forces += dragForce;
	}
	acceleration = forces * (1.0f / mass);
	//acceleration += PhysicsWorld::gravity;
	velocity += dt * acceleration;
	postion += dt * velocity;
	forces = Vector2d(0, 0);
	collider.center = postion;
	collider_aabb.updatePosition(postion);
	collider_sat.updatePosition(postion);
}
void Particle::addForce(Vector2d force)
{
	forces += force;
}
//end Particle Functions

//7//PhysicsWorld Functions
void PhysicsWorld::addParticle(Particle* particle)
{
	particles.push_back(particle);
}
void PhysicsWorld::Update(float deltaTime)
{
	list<Particle*>::iterator it;
	for (it = particles.begin(); it != particles.end(); it++)
	{
		(**it).Update(deltaTime);
	}
}
void PhysicsWorld::checkCollision()
{
	list<Particle*>::iterator it;
	list<Particle*>::iterator begin = particles.begin();
	list<Particle*>::iterator beforEnd = particles.end();;
	beforEnd--;
	for (it = particles.begin(); it != beforEnd; it++)
	{
		Particle* p1 = *it;
		Collider c1 = p1->collider;
		list<Particle*>::iterator itt;
		itt = particles.begin();
		itt++;
		for (; itt != particles.end(); itt++)
		{
			Particle* p2 = *itt;
			Collider c2 = p2->collider;
			if (c1.checkCollision(c2))
			{
				Vector2d v1 = p1->velocity;
				float m1 = p1->mass;
				Vector2d x1 = p1->postion;
				Vector2d v2 = p2->velocity;
				float m2 = p2->mass;
				Vector2d x2 = p2->postion;
				// collision direction 
				Vector2d normal1 = x1 - x2;
				float dot1 = (v1 - v2).Dot(normal1);
				float normalMag = normal1.getMagnitude();
				dot1 /= normalMag;
				dot1 /= normalMag;
				Vector2d normal2 = x2 - x1;
				float dot2 = (v2 - v1).Dot(normal2);
				normalMag = normal2.getMagnitude();
				dot2 /= normalMag;
				dot2 /= normalMag;
				v1.print();
				v2.print();
				v1 = v1 - (dot1 * ((2 * m2) / (m1 + m2))) * normal1;
				v2 = v2 - (dot2 * ((2 * m1) / (m1 + m2))) * normal2;
				v1.print();
				v2.print();
				/*p1->postion += 5 * normal1.getNormalized();
				p2->postion += 5 * normal2.getNormalized();*/
				p1->velocity = v1;
				p2->velocity = v2;
				//cout << "collision" << endl;
			}
		}
	}
}
void PhysicsWorld::checktwoCircleCollision()
{
	list<Particle*>::iterator it;
	list<Particle*>::iterator begin = particles.begin();
	list<Particle*>::iterator beforEnd = particles.end();;
	beforEnd--;
	for (it = particles.begin(); it != beforEnd; it++)
	{
		Particle* p1 = *it;
		Collider c1 = p1->collider;
		list<Particle*>::iterator itt;
		itt = particles.begin();
		itt++;
		for (; itt != particles.end(); itt++)
		{
			Particle* p2 = *itt;
			Collider c2 = p2->collider;

			if (c1.checkCollision(c2))
			{
				cout << "collision" << endl;
			}
		}
	}
}
void PhysicsWorld::checkAABBCollision()
{
	list<Particle*>::iterator it;
	list<Particle*>::iterator begin = particles.begin();
	list<Particle*>::iterator beforEnd = particles.end();;
	beforEnd--;
	for (it = particles.begin(); it != beforEnd; it++)
	{
		Particle* p1 = *it;
		AABBCollider c1 = p1->collider_aabb;
		list<Particle*>::iterator itt;
		itt = particles.begin();
		itt++;
		for (; itt != particles.end(); itt++)
		{
			Particle* p2 = *itt;
			AABBCollider c2 = p2->collider_aabb;
			if (c1.checkCollision(c2))
			{
				cout << "collision" << endl;
			}
		}
	}
}
void PhysicsWorld::checkSatCollision()
{
	list<Particle*>::iterator it;
	list<Particle*>::iterator begin = particles.begin();
	list<Particle*>::iterator beforEnd = particles.end();;
	beforEnd--;
	for (it = particles.begin(); it != beforEnd; it++)
	{
		Particle* p1 = *it;
		SATCollider c1 = p1->collider_sat;
		list<Particle*>::iterator itt;
		itt = particles.begin();
		itt++;
		for (; itt != particles.end(); itt++)
		{
			Particle* p2 = *itt;
			SATCollider c2 = p2->collider_sat;
			if (c1.checkCollision(c2))
			{
				cout << "collision" << endl;
			}
		}
	}
}
const Vector2d PhysicsWorld::gravity = Vector2d(0, 9.8f);
//end PhysicsWorld Functions

//8//Body Functions
Body::Body(Vector2f pos, uint64_t m, Vector2f dir, bool with_path) //Different color encodings for different masses (radius represents mass)
	: position(pos), direction(dir), mass(m), show_path(with_path), path(LinesStrip)
{
	// Radius
	radius = radiusForMass(mass);
	if (radius < 1)
	{
		radius = 1;
	}
	Color color(Color::Blue);
	//Color color(Color(135, 206, 235)); //Decimal Code (R,G,B) for skyblue screen
	Color clear_yellow(212, 193, 106);
	Color brown(175, 75, 0);
	if (mass <= 10000)
	{
		color = interpolate(Color::Blue, clear_yellow, mass / 10000.0f);
	}
	else if (mass <= 100000)
	{
		color = interpolate(clear_yellow, brown, mass / 100000.0f);
	}
	else if (mass >= 1000000)
	{
		color = Color::Red;
	}
	else
	{
		color = interpolate(brown, Color::Red, mass / 1000000.0f);
	}
	shape.setRadius(radius);
	shape.setFillColor(color);
}
void Body::move(float dt)
{
	position += direction * dt;
	if (show_path)
	{
		path.append(Vertex(getCenter(), shape.getFillColor()));
	}
}
void Body::applyGravityOf(const Body& b, float dt)
{
	float r = getDistanceTo(b);
	if (r <= b.radius)
	{
		return;
	}
	float F = (G * mass * b.mass) / (r * r); // Newton's law of universal gravitation
	// Make the force proportional to the mass //Remember when we set G to 1 in the header file xd (for simplicity)
	F /= mass;
	// Get the unit vector to the other body
	Vector2f to_Body(b.getCenter() - getCenter());
	to_Body = to_Body / r;
	// Apply the force in the direction of the other body
	direction += (to_Body * F) * dt;
}
inline
float Body::getDistanceTo(const Body& b) const
{
	Vector2f c = b.getCenter() - getCenter();
	return sqrt(c.x * c.x + c.y * c.y); //2D space distance magnitude
}
inline
Vector2f Body::getCenter() const
{
	return Vector2f(position.x + radius, position.y + radius);
}
void Body::draw(RenderWindow& window)
{
	shape.setPosition(position);
	window.draw(shape);
	if (show_path)
	{
		window.draw(path);
	}
}
bool Body::collideWith(const Body& b) const
{
	Vector2f a = getCenter();
	Vector2f c = b.getCenter();
	float d = (a.x - c.x) * (a.x - c.x) + (a.y - c.y) * (a.y - c.y);
	return (d <= (radius + b.radius) * (radius + b.radius));
}
bool Body::contains(const Vector2f& point) const
{
	Vector2f center(getCenter());
	return (point.x - center.x) * (point.x - center.x)
		+ (point.y - center.y) * (point.y - center.y) <= radius * radius;
}
void Body::setPathEnabled(bool state)
{
	show_path = state;
	if (!show_path)
	{
		path.clear();
	}
}
VertexArray Body::getPath() const
{
	return path;
}
#define M_PI 3.14
float Body::radiusForMass(uint64_t mass)
{
	float volume = mass / DENSITY;
	return cbrt((3 * volume) / (4 * M_PI));
}
//end Body Functions

//9//Universe Functions
Universe::Universe()
	: old_paths(LinesStrip), show_path(false)
{
}
void Universe::addPlanet(Vector2f position, uint64_t m, Vector2f dir)
{
	planets.push_back(Body(position, m, dir, show_path));
}
void Universe::addPlanet(Body p)
{
	p.setPathEnabled(show_path);
	planets.push_back(p);
}
//Protoplanetary disk
//is a rotating circumstellar disc of dense gas and dust surrounding a young newly formed star, a T Tauri star, or Herbig Ae/Be star.
void Universe::createProtodisk(const int number, const int radius, const int mass, const sf::Vector2f& position)
{
	std::uniform_real_distribution<float> distribution(0.0f, float(radius));
	auto random = std::bind(distribution, rng);
	for (int i = 0; i < number; i++)
	{
		float t = 2 * M_PI * random();
		float u = random() + random();
		float r = (u > 1 ? 2 - u : u);
		Vector2f pos(r * cos(t), r * sin(t));
		float len = sqrt(pos.x * pos.x + pos.y * pos.y);
		pos += position;
		Vector2f dir = tangentThroughPoint(position, pos);
		dir *= interpolate(-5.0f, 60.0f, len / radius);
		Body p(pos, mass, dir, show_path);
		planets.push_back(p);
	}
}
void Universe::togglePath()
{
	show_path = !show_path;
	for (Body& b : planets)
	{
		b.setPathEnabled(show_path);
	}
	if (!show_path)
	{
		old_paths.clear();
	}
}
uint64_t Universe::getPlanetNumber() const
{
	return planets.size();
}
bool Universe::isPathEnabled() const
{
	return show_path;
}
void Universe::move(float delta_t)
{
	for (unsigned int i = 0; i < planets.size(); i++)
	{
		for (unsigned int j = 0; j < planets.size(); j++)
		{
			if (i == j)
				continue;
			if (planets[i].collideWith(planets[j]))
			{
				Body p = combinedPlanets(planets[i], planets[j]);
				p.setPathEnabled(show_path);
				savePlanetPath(planets[i]);
				savePlanetPath(planets[j]);
				int a = max(i, j), b = min(i, j);
				planets.erase(planets.begin() + a);
				planets.erase(planets.begin() + b);
				planets.push_back(p);
			}
			else
			{
				planets[i].applyGravityOf(planets[j], delta_t);
			}
			if (i >= planets.size())
				break;
		}
	}
	for (Body& b : planets)
	{
		b.move(delta_t);
	}
}
void Universe::draw(RenderWindow& window)
{
	for (Body& b : planets)
	{
		b.draw(window);
	}

	if (show_path)
	{
		for (VertexArray& path : old_paths)
		{
			window.draw(path);
		}
	}
}
void Universe::eraseAt(const Vector2f& pos)
{
	for (unsigned int i = 0; i < planets.size(); i++)
	{
		if (planets[i].contains(Vector2f(pos.x, pos.y)))
		{
			planets.erase(planets.begin() + i);
			cout << "Successfully erased one planet from existence!" << endl;
		}
	}
}
void Universe::eraseAll()
{
	planets.clear();
	old_paths.clear();
}
void Universe::savePlanetPath(const Body& b)
{
	old_paths.push_back(b.getPath());
}
// helper functions
Body combinedPlanets(const Body& a, const Body& b)
{
	Vector2f p1m1 = float(a.mass) * a.direction;
	Vector2f p2m2 = float(b.mass) * b.direction;
	uint64_t total_mass(a.mass + b.mass);
	Vector2f direction = (p1m1 + p2m2) / float(total_mass);
	Vector2f pos = (a.mass >= b.mass ? a.position :
		b.position);

	return Body(pos, total_mass, direction, false);
}
Vector2f tangentThroughPoint(Vector2f circle_center, Vector2f point)
{
	Vector2f r(point - circle_center);
	Vector2f tangent(r.y, -r.x);
	tangent /= sqrt(r.x * r.x + r.y * r.y); // unit vector = vec/vec.length
	return tangent;
}
//end Universe Functions

//10//Interface Functions
Interface::Interface(RenderWindow& win, Universe& verse)
	: modifiers{ false, false, false, false },
	universe(verse), window(win),
	view(Vector2f(0, 0), Vector2f(win.getSize())),
	base_line(BASE_LINE), line(Lines, 2),
	mass(MASS), num_planets(0)
{
	window.setView(view);
	line[0].color = Color::Magenta;
	line[1].color = Color::Red;
	base_line.setFillColor(Color::Magenta);
	font.loadFromFile("DejaVuSans.ttf"); //Fancy text to print on window to user
	mass_text.setFont(font); //print white text on window
	// mass_text.setFillColor(Color::White);
	num_planets_text.setFont(font);
	// num_planets_text.setFillColor(Color::White);
	paths_text.setFont(font);
}
void Interface::handle_event(Event evt)
{
	if (evt.type == Event::Closed) //Close window
	{
		window.close();
	}
	else if (evt.type == Event::Resized) //Yes you can zoom in and out and resize the window to see SPACEEEEE
	{
		view.setSize(evt.size.width, evt.size.height);
		window.setView(view);
	}
	else if (evt.type == Event::KeyPressed) //Awaiting hey press from user
	{
		if (evt.key.code == Keyboard::Escape) //Closes on 'Esc' key
		{
			window.close();
		}
		switch (evt.key.code)
		{
		case Keyboard::PageUp: //Increasing mass on 'PageUp' & 'Add' keys, increments of 10x
		case Keyboard::Add:
			mass *= 10;
			break;
		case Keyboard::PageDown: //Decreasing mass on 'PageDown' & 'Subtract' keys, increments of /10
		case Keyboard::Subtract:
			if (mass > 10)
			{
				mass /= 10;
			}
			else
			{
				mass = MASS; //Default (M=10)
			}
			break;
		case Keyboard::P:			//See the paths of movement 'P' key
			universe.togglePath();
			break;
		case Keyboard::Space:		//Toggle placing planets 'Space' key
			modifiers.is_placing = false;
			break;
		case Keyboard::LControl:	//Toggle control of window 'LControl' key
			modifiers.control = true;
			break;
		case Keyboard::LShift:		//Toggle Shifting window 'LShift' key
			modifiers.shift = true;
			break;
		case Keyboard::C:			// Create a cluster or orbiting planets in 2D space Protodisk 'C' Key for Cluster
			universe.createProtodisk(CLUSTER_NUMBER, CLUSTER_RADIUS, CLUSTER_MASS, view.getCenter());
			break;
		case Keyboard::Delete:		//Delete the universe from existence, return to monke! xd 'Delete' key
			universe.eraseAll();
			break;
		default:
			break;
		}
	}
	else if (evt.type == Event::KeyReleased)
	{
		switch (evt.key.code)
		{
		case Keyboard::LControl:	//Toggle control of window 'LControl' key
			modifiers.control = false;
			break;
		case Keyboard::LShift:		//Toggle Shifting window 'LShift' key
			modifiers.shift = false;
			break;
		default:
			break;
		}
	}
	else if (evt.type == Event::MouseButtonPressed)
	{
		if (modifiers.shift)
		{
			universe.eraseAt(window.mapPixelToCoords(Vector2i(evt.mouseButton.x, evt.mouseButton.y)));
		}
		else if (modifiers.control)
		{
			modifiers.is_moving = true;
		}
		else
		{
			modifiers.is_placing = true;
			line[0].position = window.mapPixelToCoords(Vector2i(evt.mouseButton.x, evt.mouseButton.y));
			line[1].position = line[0].position;
			start = Vector2f(evt.mouseButton.x, evt.mouseButton.y);
			end = start;
			Vector2f base_pos = Vector2f(line[0].position.x - BASE_LINE, line[0].position.y - BASE_LINE);
			base_line.setPosition(base_pos);
		}
	}
	else if (evt.type == Event::MouseMoved)	//Handling mouse movement
	{
		if (modifiers.is_placing)
		{
			line[1].position = window.mapPixelToCoords(Vector2i(evt.mouseMove.x, evt.mouseMove.y));
			end = Vector2f(evt.mouseMove.x, evt.mouseMove.y);
		}
		if (modifiers.is_moving)
		{
			view.move(mousePosition - window.mapPixelToCoords(Vector2i(evt.mouseMove.x, evt.mouseMove.y)));
			window.setView(view);
		}
		mousePosition = window.mapPixelToCoords(Vector2i(evt.mouseMove.x, evt.mouseMove.y));
	}
	if (evt.type == Event::MouseButtonReleased)
	{
		if (modifiers.is_placing)
		{
			modifiers.is_placing = false;
			Vector2f direction = end - start;
			float radius = Body::radiusForMass(mass);
			Vector2f position = Vector2f(line[0].position.x - radius,
				line[0].position.y - radius);
			universe.addPlanet(position, mass, direction);
		}
		if (modifiers.is_moving)
		{
			modifiers.is_moving = false;
		}
	}
	else if (evt.type == Event::MouseWheelMoved) //Lets implement zooming in space
	{
		float factor = (evt.mouseWheel.delta < 1 ? 1.10 : 0.90);
		view.zoom(factor);
		window.setView(view);
	}
	mass_text.setPosition(Vector2f());
	uint32_t csize = mass_text.getCharacterSize();
	num_planets_text.setPosition(Vector2f(0, csize));
	paths_text.setPosition(Vector2f(0, csize * 2));
}
void Interface::draw() //Now lets add some graphics to the user GUI with a nice font (drawing to window)
{
	if (modifiers.is_placing)
	{
		window.draw(line);
		window.draw(base_line);
	}
	num_planets = universe.getPlanetNumber();
	mass_text.setString("Mass: " + std::to_string(mass));
	num_planets_text.setString("Number of planets: " + std::to_string(num_planets));
	paths_text.setString(std::string("Paths: ") + (universe.isPathEnabled() ? "True" : "False"));
	// Reset view to draw text in window coordinates
	View v = window.getView();
	window.setView(window.getDefaultView());
	window.draw(mass_text);
	window.draw(num_planets_text);
	window.draw(paths_text);
	window.setView(v);
}
//end Interface Functions

//end Functions

//I hope this works ~Rwhooshh

#endif // SOLARSPACESYSTEM_FUNCTIONS_HPP