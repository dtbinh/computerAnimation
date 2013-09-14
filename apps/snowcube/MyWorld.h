#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include <Eigen/Dense>
#include "force.h"
#include "particleManagerSnowCube.h"

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

    Eigen::Vector3d getCubePosition() {
        return mCubePosition;
    }

	void moveContainer (double x, double y );

    // TODO: your simulation code goes here
    void simulate();
	void enableWind() { mForces[1]->enable( true ); }
	void disableWind() { mForces[1]->enable( false ); }
    
 protected:
    std::vector<Particle*> mParticles;
    Eigen::Vector3d mCubePosition;
	std::vector<IForce*> mForces;
	ParticleManagerSnowCube mParticleManager;
};

#endif
