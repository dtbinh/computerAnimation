#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include "particleManagerGalileo.h"
#include "force.h"

class Particle;

class MyWorld {
 public:
    MyWorld(int _numParticles);

    virtual ~MyWorld();

    int getNumParticles() {
        return mParticles.size();
    }

    Particle* getParticle(int _index) {
        return mParticles[_index];
    }

    // TODO: your simulation code goes here
    void simulate();
    
 protected:
    std::vector<Particle*> mParticles;
	std::vector<IForce*> mForces;
	ParticleManagerGalileo mParticleManager;
};

#endif
