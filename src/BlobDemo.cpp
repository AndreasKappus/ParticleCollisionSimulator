/*
 * The Blob demo.
 *
 */
#include <GL/glut.h>
#include "app.h"
#include "coreMath.h"
#include "pcontacts.h"
#include "pworld.h"
#include <stdio.h>
#include <cassert>
#include<iostream>
#include<cstdlib>
#include<time.h>

int BLOB_COUNT;
float RADIUS;
#define PLATFORM_COUNT 7

const Vector2 Vector2::GRAVITY = Vector2(0, -9.81);

/**
 * Platforms are two dimensional: lines on which the
 * particles can rest. Platforms are also contact generators for the physics.
 */

class Platform : public ParticleContactGenerator
{
public:
	Vector2 start;
	Vector2 end;
	float restitution;
	/**
	 * Holds a pointer to the particles we're checking for collisions with.
	 */
	Particle* particles;

	virtual unsigned addContact(
		ParticleContact* contact,
		unsigned limit
	) const;
};

unsigned Platform::addContact(ParticleContact* contact,
	unsigned limit) const
{

	//const static float restitution = 0.8f;
	const static float restitution = 1.0f;
	unsigned used = 0;

	for (unsigned i = 0; i < BLOB_COUNT; i++)
	{
		if (used >= limit) return used;
		
		// Check for penetration
		Vector2 toParticle = particles[i].getPosition() - start;
		Vector2 lineDirection = end - start;

		float projected = toParticle * lineDirection;
		float platformSqLength = lineDirection.squareMagnitude();
		float squareRadius = particles[i].getRadius() * particles[i].getRadius();

		if (projected <= 0)
		{

			// The blob is nearest to the start point
			if (toParticle.squareMagnitude() < squareRadius)
			{
				// We have a collision
				contact->contactNormal = toParticle.unit();
				contact->restitution = restitution;
				contact->particle[0] = particles + i;
				contact->particle[1] = 0;
				contact->penetration = particles[i].getRadius() - toParticle.magnitude();
				used++;
				contact++;
			}

		}
		else if (projected >= platformSqLength)
		{
			// The blob is nearest to the end point
			toParticle = particles[i].getPosition() - end;
			if (toParticle.squareMagnitude() < squareRadius)
			{
				// We have a collision
				contact->contactNormal = toParticle.unit();
				contact->restitution = restitution;
				contact->particle[0] = particles + i;
				contact->particle[1] = 0;
				contact->penetration = particles[i].getRadius() - toParticle.magnitude();
				used++;
				contact++;
			}
		}
		else
		{
			// the blob is nearest to the middle.
			float distanceToPlatform = toParticle.squareMagnitude() - projected * projected / platformSqLength;
			if (distanceToPlatform < squareRadius)
			{
				// We have a collision
				Vector2 closestPoint = start + lineDirection * (projected / platformSqLength);

				contact->contactNormal = (particles[i].getPosition() - closestPoint).unit();
				contact->restitution = restitution;
				contact->particle[0] = particles + i;
				contact->particle[1] = 0;
				contact->penetration = particles[i].getRadius() - sqrt(distanceToPlatform);
				used++;
				contact++;
			}
		}
	}

	

	return used;
}


class BlobDemo : public Application
{
	Particle* blobs;

	int num_platforms;

	Platform* platforms;

	ParticleWorld world;
	ParticleCollision* pCollision;

public:
	/** Creates a new demo object. */
	BlobDemo();
	virtual ~BlobDemo();

	/** Returns the window title for the demo. */
	virtual const char* getTitle();

	/** Display the particles. */
	virtual void display();

	/** Update the particle positions. */
	virtual void update();

	float randNumGen(float min, float max);

	int particleMenu(int particleNum);

};

float BlobDemo::randNumGen(float min, float max)
{

	return((float)rand() / RAND_MAX) * (max - min) + min;
}



// Method definitions
BlobDemo::BlobDemo() :world(PLATFORM_COUNT + BLOB_COUNT * BLOB_COUNT, PLATFORM_COUNT + BLOB_COUNT)
{	
	std::cout << "Please enter amount of particles: ";
	std::cin >> BLOB_COUNT;
	while (BLOB_COUNT <= 4)
	{
		std::cout << "Not enough particles! Please enter more than 4 particles: ";
		std::cin >> BLOB_COUNT;
	}

	std::cout << "please set radius for all particles: ";
	std::cin >> RADIUS;
	while (RADIUS < 0.1)
	{
		std::cout << "Radius cannot be 0! please enter larger radius size: ";
		std::cin >> RADIUS;
	}

	width = 400; height = 400;
	nRange = 100.0;
	float margin = 0.95;
	// Create the blob storage
	blobs = new Particle[BLOB_COUNT];

	float standard_restitution = 1;
	// Create the platform
	platforms = new Platform[PLATFORM_COUNT];
	

	platforms[0].start = Vector2(-50.0, 0.0);
	platforms[0].end = Vector2(50.0, -10.0);
	platforms[0].restitution = 0.6;

	platforms[1].start = Vector2(-nRange * margin, -nRange * margin);
	platforms[1].end = Vector2(nRange * margin, -nRange * margin);
	platforms[1].restitution = standard_restitution;

	platforms[2].start = Vector2(-nRange * margin, nRange * margin);
	platforms[2].end = Vector2(nRange * margin, nRange * margin);
	platforms[2].restitution = standard_restitution;

	platforms[3].start = Vector2(-nRange * margin, -nRange * margin);
	platforms[3].end = Vector2(-nRange * margin, nRange * margin);
	platforms[3].restitution = standard_restitution;

	platforms[4].start = Vector2(nRange * margin, -nRange * margin);
	platforms[4].end = Vector2(nRange * margin, nRange * margin);
	platforms[4].restitution = standard_restitution;

	platforms[5].start = Vector2(80.0, -40.0);
	platforms[5].end = Vector2(0.0, -70.0);
	platforms[5].restitution = 0.8;


	platforms[6].start = Vector2(-20.0, -80.0);
	platforms[6].end = Vector2(-80.0, -50.0);
	platforms[6].restitution = 0.95;


	// Make sure the platform knows which particle it should collide with.
	for (unsigned i = 0; i < PLATFORM_COUNT; ++i)
	{
		platforms[i].particles = blobs;
		world.getContactGenerators().push_back(platforms + i);
	}

	for (unsigned i = 0; i < BLOB_COUNT; i++)
	{
		
		// Create the blob
		//blobs[i].setPosition(randNumGen(-nRange, nRange), 90.0);
		blobs[i].setPosition(randNumGen(-50, 50), 90.0);
		//blobs[i].setRadius(4);
		blobs[i].setRadius(RADIUS);
		blobs[i].setVelocity(0, 0);
		blobs[i].setDamping(0.5);
		blobs[i].setAcceleration(Vector2::GRAVITY * 5.0 * (i+1));
		blobs[i].setMass(10.0f);
		blobs[i].clearAccumulator();
		world.getParticles().push_back(blobs + i);
	}
	
	// constructor to create a new quad tree with the blobs being set to the particles to enable the quad tree method to work
	pCollision = new ParticleCollision(blobs, Vector2(nRange * 2.0, nRange * 2.0), BLOB_COUNT);
	pCollision->particles = blobs;
	// allow for particle to particle collision with the quad tree 
	world.getContactGenerators().push_back(pCollision);
	
}


BlobDemo::~BlobDemo()
{
	delete blobs;
}

void BlobDemo::display()
{
	Application::display();


	glBegin(GL_LINES);
	glColor3f(0, 1, 1);

	for (unsigned i = 0; i < PLATFORM_COUNT; i++)
	{
		const Vector2 &p0 = platforms[i].start;
		const Vector2 &p1 = platforms[i].end;
		glVertex2f(p0.x, p0.y);
		glVertex2f(p1.x, p1.y);
	}
	glEnd();


	for (unsigned i = 0; i < BLOB_COUNT; i++)
	{
		glColor3f((i % 2) ? 0 : 1, (i % 2) ? 1 : 0, 0);

		const Vector2& p = blobs[i].getPosition();
		glPushMatrix();
		glTranslatef(p.x, p.y, 0);
		glutSolidSphere(blobs[i].getRadius(), 12, 12);
		glPopMatrix();
	}

	// calls the grid and collision line methods to visualise the grids and the collisions within quadrants
	pCollision->treeGrid();
	pCollision->collisionLines();
	
	
	glutSwapBuffers();

}

void BlobDemo::update()
{
	// calls the particle collision update method within the blob demo update, it seems to only work this way so PLZ DONT REMOVE
	pCollision->update();

	// Recenter the axes
	float duration = timeinterval / 1000;
	// Run the simulation
	world.runPhysics(duration);


	Application::update();
}

const char* BlobDemo::getTitle()
{
	return "Blob Demo";
}


/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
	return new BlobDemo();
}