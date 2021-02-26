/*
 * Interface file for the contact resolution system for particles.
 *
 */

#ifndef PCONTACTS_H
#define PCONTACTS_H

#include "particle.h"
#include<vector>
#include<iostream>
#include<list>
#include <GL/glut.h>


using namespace std;

    class ParticleContactResolver;

    /**
     * A Contact represents two objects in contact (in this case
     * ParticleContact representing two Particles). 
     */
    class ParticleContact
    {
        /**
         * The contact resolver object needs access into the contacts to
         * set and effect the contact.
         */
        friend ParticleContactResolver;

    public:
        /**
         * Holds the particles that are involved in the contact. The
         * second of these can be NULL, for contacts with the scenery.
         */
        Particle* particle[2];

        /**
         * Holds the normal restitution coefficient at the contact.
         */
        float restitution;

        /**
         * Holds the direction of the contact in world coordinates.
         */
        Vector2 contactNormal;

        /**
         * Holds the depth of penetration at the contact.
         */
        float penetration;

		// Holds amount each particle is moved by during interpenetration
		Vector2 particleMovement[2];


    protected:
        /**
         * Resolves this contact, for both velocity and interpenetration.
         */
        void resolve(float duration);

        /**
         * Calculates the separating velocity at this contact.
         */
        float calculateSeparatingVelocity() const;

    private:
        /**
         * Handles the impulse calculations for this collision.
         */
        void resolveVelocity(float duration);

		void resolveInterpenetration(float duration);

    };

    /**
     * The contact resolution routine for particle contacts. One
     * resolver instance can be shared for the whole simulation.
     */
    class ParticleContactResolver
    {
    protected:
        /**
         * Holds the number of iterations allowed.
         */
        unsigned iterations;

        /**
         * This is a performance tracking value - we keep a record
         * of the actual number of iterations used.
         */
        unsigned iterationsUsed;

    public:
        /**
         * Creates a new contact resolver.
         */
        ParticleContactResolver(unsigned iterations);

        /**
         * Sets the number of iterations that can be used.
         */
        void setIterations(unsigned iterations);

        /**
         * Resolves a set of particle contacts for both penetration
         * and velocity.
         *
        */
        void resolveContacts(ParticleContact *contactArray,
            unsigned numContacts,
            float duration);

    };

    /**
     * This is the basic polymorphic interface for contact generators
     * applying to particles.
     */
    class ParticleContactGenerator
    {
    public:
        /**
         * Fills the given contact structure with the generated
         * contact. 
         */
        virtual unsigned addContact(ParticleContact *contact,
                                    unsigned limit) const = 0;
    };
	

	class Quadtree {
	public:
		// Quad tree constructor, takes the x and y positions along with the size dimensions
		Quadtree(float x, float y, float width, float height);

		// Destructor to remove quadrants
		~Quadtree();

		int size = 0;

		// adds particles to the quad tree, returns true if added
		bool addObject(Particle* particles);

		// gets the particles within the quad tree 
		vector<vector<Particle*>> getObjects(vector<vector<Particle*>> particleQuadrants);

		// draws a grid for the quad tree partitioning
		void grid();

		// splits the quadrant into 4 
		void splitQuadrant();

		// check if the quadrant contains particles
		bool contains(Particle* particle);
		bool split = false;

	private:
		float x;
		float y;
		float width;
		float height;
		const int limit = 4;
		vector<Particle*>objects;

		// location of particle within quadrant
		Particle* gridLocation;

		// node tree quadrants after the initial node
		Quadtree* NW = nullptr;
		Quadtree* NE = nullptr;
		Quadtree* SW = nullptr;
		Quadtree* SE = nullptr;
		

	};


	class ParticleCollision : public ParticleContactGenerator
	{
	private:
		// screensize for the initial quad tree node
		Vector2 screenSize;
		Vector2 position;
		int numParticles;
		Quadtree* initialNode = nullptr;
	public:
		
		Particle* particles;
		// constructor for blob demo class
		ParticleCollision(Particle* particle, Vector2 size, int _numParticles);

		// polymorph method from particleContactGenerator class
		virtual unsigned addContact(ParticleContact* contact, unsigned limit) const;

		// checks collisions between two spheres
		bool checkCollision(Particle* particle1, Particle* particle2, float distance) const;

		// draws lines between particles within their quadrants
		void collisionLines();

		// draws another grid within a quadrant when split
		void treeGrid();

		// create the initial node for the quad tree
		void createInitialNode();

		//  updates the quadtree structure based on particle locations within quadrants
		void update();

	};
	
#endif // CONTACTS_H