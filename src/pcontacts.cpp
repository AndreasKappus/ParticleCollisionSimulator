
#include <float.h>
#include <pcontacts.h>
#include<iostream>



// Contact implementation
void ParticleContact::resolve(float duration)
{
    resolveVelocity(duration);
	resolveInterpenetration(duration);
}



float ParticleContact::calculateSeparatingVelocity() const
{
    Vector2 relativeVelocity = particle[0]->getVelocity();
    if (particle[1]) relativeVelocity -= particle[1]->getVelocity();
    return relativeVelocity * contactNormal;
}

void ParticleContact::resolveVelocity(float duration)
{
    // Find the velocity in the direction of the contact
    float separatingVelocity = calculateSeparatingVelocity();

    // Check if it needs to be resolved
    if (separatingVelocity > 0)
    {
        // The contact is either separating, or stationary - there's
        // no impulse required.
        return;
    }

    // Calculate the new separating velocity
    float newSepVelocity = -separatingVelocity * restitution;
    float deltaVelocity = newSepVelocity - separatingVelocity;

    // We apply the change in velocity to each object in proportion to
    // their inverse mass (i.e. those with lower inverse mass [higher
    // actual mass] get less change in velocity)..
    float totalInverseMass = particle[0]->getInverseMass();
    if (particle[1]) totalInverseMass += particle[1]->getInverseMass();

    // If all particles have infinite mass, then impulses have no effect
    if (totalInverseMass <= 0) return;

    // Calculate the impulse to apply
    float impulse = deltaVelocity / totalInverseMass;

    // Find the amount of impulse per unit of inverse mass
    Vector2 impulsePerIMass = contactNormal * impulse;

    // Apply impulses: they are applied in the direction of the contact,
    // and are proportional to the inverse mass.
    particle[0]->setVelocity(particle[0]->getVelocity() +
        impulsePerIMass * particle[0]->getInverseMass()
        );
    if (particle[1])
    {
        // Particle 1 goes in the opposite direction
        particle[1]->setVelocity(particle[1]->getVelocity() +
            impulsePerIMass * -particle[1]->getInverseMass()
            );
    }
}

void ParticleContact::resolveInterpenetration(float duration)
{
	// If we don't have any penetration, skip this step.
	if (penetration <= 0) return;
	// The movement of each object is based on their inverse mass, so
	// total that.
	float totalInverseMass = particle[0]->getInverseMass();
	if (particle[1]) totalInverseMass += particle[1]->getInverseMass();
	// If all particles have infinite mass, then we do nothing
	if (totalInverseMass <= 0) return;
	// Find the amount of penetration resolution per unit of inverse mass
	Vector2 movePerIMass = contactNormal * (penetration / totalInverseMass);
	// Calculate the the movement amounts
	particleMovement[0] = movePerIMass * particle[0]->getInverseMass();
	if (particle[1]) {
		particleMovement[1] = movePerIMass * -particle[1]->getInverseMass();
	}
	else {
		particleMovement[1].clear();
	}
	// Apply the penetration resolution
	particle[0]->setPosition(particle[0]->getPosition() + particleMovement[0]);
	if (particle[1]) {
		particle[1]->setPosition(particle[1]->getPosition() + particleMovement[1]);
	}

}

ParticleContactResolver::ParticleContactResolver(unsigned iterations)
:
iterations(iterations)
{
}

void ParticleContactResolver::setIterations(unsigned iterations)
{
    ParticleContactResolver::iterations = iterations;
}

void ParticleContactResolver::resolveContacts(ParticleContact *contactArray,
                                              unsigned numContacts,
                                              float duration)
{
    unsigned i;

    iterationsUsed = 0;
    while(iterationsUsed < iterations)
    {
        // Find the contact with the largest closing velocity;
        float max = DBL_MAX;
        unsigned maxIndex = numContacts;
        for (i = 0; i < numContacts; i++)
        {
            float sepVel = contactArray[i].calculateSeparatingVelocity();
            if (sepVel < max &&
                (sepVel < 0 || contactArray[i].penetration > 0))
            {
                max = sepVel;
                maxIndex = i;
            }
        }
         //Do we have anything worth resolving?
        if (maxIndex == numContacts) break;

        // Resolve this contact
        contactArray[maxIndex].resolve(duration);
        iterationsUsed++;
    }

}

// ----------------- QUAD TREE CODE ----------------------

Quadtree::Quadtree(float x, float y, float width, float height)
{
	this->x = x;
	this->y = y;
	this->width = width;
	this->height = height;
}


Quadtree::~Quadtree()
{
	if (NW != nullptr)
		delete NW;
	if (NE != nullptr)
		delete NE;
	if (SW != nullptr)
		delete SW;
	if (SE != nullptr)
		delete SE;
}


bool Quadtree::addObject(Particle* particles)
{
	
	if (!this->contains(particles))
		return false;

	// if there is still capacity for more particles in a quadrant, push more particles into the quadrant
	if (this->objects.capacity() < this->limit)
	{
		this->objects.push_back(particles);
		return true;
	}

	// if there's no more capacity, split the quadrant
	else {
		if (!this->split) {
			splitQuadrant();

			for (int i = 0; i < this->limit; i++)
				this->addObject(objects[i]);

			this->objects.clear();
		}
		// return true if particles were added to the specified quadrants
		if (this->NW->addObject(particles))
			return true;
		if (this->NE->addObject(particles))
			return true;
		if (this->SW->addObject(particles))
			return true;
		if (this->SE->addObject(particles))
			return true;

		return false;
	}
}


void Quadtree::splitQuadrant()
{
	// splits the specified quadrants into 4, the quadrant size is the width and height of the quadrant divided by 2
	this->NW = new Quadtree(x - (this->width / 4), y + (this->height / 4), (width / 2), (height / 2));
	this->NE = new Quadtree(x + (this->width / 4), y + (this->height / 4), (width / 2), (height / 2));
	this->SW = new Quadtree(x - (this->width / 4), y - (this->height / 4), (width / 2), (height / 2));
	this->SE = new Quadtree(x + (this->width / 4), y - (this->height / 4), (width / 2), (height / 2));
	// for future checks
	this->split = true;
}


vector<vector<Particle*>> Quadtree::getObjects(vector<vector<Particle*>> particleQuadrants)
{
	// gets particles within quadrants if the quad tree has been divided
	if (this->split == true)
	{
		particleQuadrants = NW->getObjects(particleQuadrants);
		particleQuadrants = NE->getObjects(particleQuadrants);
		particleQuadrants = SW->getObjects(particleQuadrants);
		particleQuadrants = SE->getObjects(particleQuadrants);

		return particleQuadrants;
	}
	else
	{
		particleQuadrants.push_back(this->objects);
		return particleQuadrants;
	}
}


bool Quadtree::contains(Particle* particle)
{
	Vector2 pos = particle->getPosition();
	float radius = particle->getRadius();

	// ensures the particles are within the collision bounds for the quadrants
	if (pos.y + radius >= this->y - (this->height / 2))
	{
		if (pos.y - radius <= this->y + (this->height / 2))
		{
			if (pos.x + radius >= this->x - (this->width / 2))
			{
				if (pos.x - radius <= this->x + (this->width / 2))
					return true;
			}
		}
	}

	return false;
}


void Quadtree::grid()
{
	// initial grid for quadrants
	glLineWidth(1);
	glColor3f(1.0f, 1.0f, 1.0f);

	glBegin(GL_LINES);
	glVertex2f(this->x - (width / 2), this->y);
	glVertex2f(this->x + (width / 2), this->y);
	glVertex2f(this->x, this->y - (height / 2));
	glVertex2f(this->x, this->y + (height / 2));
	glEnd();

		// draws another grid inside the quadrants

		if (this->NW != nullptr)
		{
			if (this->NW->NW != nullptr)
				this->NW->grid();
		}
		if (this->NE != nullptr)
		{
			if (this->NE->NE != nullptr)
				this->NE->grid();
		}
		if (this->SW != nullptr)
		{
			if (this->SW->SW != nullptr)
				this->SW->grid();
		}
		if (this->SE != nullptr)
		{
			if (this->SE->SE != nullptr)
				this->SE->grid();
		}
		
	
	

}

// ------------- PARTICLE COLLISION CODE MODIFIED TO WORK WITH QUAD TREE ------------------------

ParticleCollision::ParticleCollision(Particle* particle,Vector2 size, int _numParticles)
{
	this->screenSize = size;
	this->particles = particle;
	this->numParticles = _numParticles;
}

void ParticleCollision::update()
{
	createInitialNode();
}

void ParticleCollision::createInitialNode()
{
	if (initialNode != nullptr)
		delete initialNode;

	// create a new quadtree with the quadtree constructor
	initialNode = new Quadtree(0, 0, screenSize.x, screenSize.y);

	// add particles to the quad tree nodes
	for (int i = 0; i < numParticles; i++)
	{
		initialNode->addObject(&particles[i]);
	}
}

unsigned ParticleCollision::addContact(ParticleContact* contact, unsigned limit) const
{
	unsigned used = 0;
	const static float restitution = 1.0f;
	// used to get the particle locations within the quadrants
	vector<vector<Particle*>> gridLocation;
	gridLocation = initialNode->getObjects(gridLocation);

	// iterate throught the 2d vector elements for the particle grid locations
	for (vector<vector<Particle*>>::iterator i = gridLocation.begin(); i != prev(gridLocation.end()); i++)
	{
		// if there's 2 particles in sector
		if ((*i).size() >= 2)
		{
			// two vector iterators for the particles within the vector which is used for collision handling
			for (vector<Particle*>::iterator j = (*i).begin(); j != prev((*i).end()); j++)
			{
				if (used >= limit) return used;
				for (vector<Particle*>::iterator k = next(j); k != (*i).end(); k++)
				{		

					Vector2 distance = (*j)->getPosition() - (*k)->getPosition();
					float size = distance.magnitude();
					float radius1 = (*j)->getRadius();
					float radius2 = (*k)->getRadius();
					if (checkCollision((*j), (*k), size))
					{
						// handle the particle contact when collisions are made
						contact->contactNormal = distance.unit();
						contact->restitution = restitution;
						contact->particle[0] = (*j);
						contact->particle[1] = (*k);
						contact->penetration = (radius1 + radius2) - size;
						used++;
						contact++;
					}
				}
			}
		}
	}
	return used;
}


bool ParticleCollision::checkCollision(Particle* particle1, Particle* particle2, float distance) const
{
	// if the distance between the centre of two spheres is less than the combined radii, 
	// then the spheres have collided
			if (distance <= particle1->getRadius() + particle2->getRadius())
			{
				return true;
			}
			else
			{
				return false;
			}
}

void ParticleCollision::collisionLines()
{
	// get the location of the particles within the quadrants, so collisions can be partitioned to their respective areas
	vector<vector<Particle*>> gridLocation;
	gridLocation = initialNode->getObjects(gridLocation);

	for (vector<vector<Particle*>>::iterator i = gridLocation.begin(); i != prev(gridLocation.end()); i++)
	{
		if ((*i).size() >= 2)
		{
			// iterate through the vector of particles with their grid locations to draw collision lines for the particles within their quadrants

			for (vector<Particle*>::iterator j = (*i).begin(); j != prev((*i).end()); j++)
			{
				for (vector<Particle*>::iterator k = next(j); k != (*i).end(); k++)
				{
					glColor3f(1.0f, 0.0f, 0.0f);
					glBegin(GL_LINES);
					glVertex2f((*j)->getPosition().x, (*j)->getPosition().y);
					glVertex2f((*k)->getPosition().x, (*k)->getPosition().y);
					glEnd();
				}
			}
		}
	}
}

void ParticleCollision::treeGrid()
{
	// calls the grid method from the quad tree class to draw more grids when the quadrants are divided
	initialNode->grid();
}



