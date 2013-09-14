#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include <Eigen/Dense>
#include "particleManagerTinkerToy.h"
#include "constraintManager.h"
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
    void simulate(double x, double y);
	void onLeftMouseClick( double x, double y );
	void onRightMouseClick( double x, double y );
	void resumeSimulation();
	void draw(renderer::RenderInterface* pRI);
	void toggleSpringMode() { mFSpringMode = !mFSpringMode; }
	void onLeftMouseReleased() { mParticleManager.removeSpring(); mFSpringMode = false; mFShowSpring = false; }
	void enableGravity() { mForces[0]->enable( true ); }
	void disableGravity() { mForces[0]->enable( false ); }
	void enableDamping() { mForces[1]->enable( true ); }
	void disableDamping() { mForces[1]->enable( false ); }
    
 protected:
    std::vector<Particle*> mParticles;
	std::vector<IForce*> mForces;
	ParticleManagerTinkerToy mParticleManager;
	ConstraintManager mConstraintManager;

	//States for adding particles and constraints
	bool mFPauseSimulation;
	bool mFSpringMode;
	bool mFShowSpring;
	std::auto_ptr<Particle> m_newParticle;
};

#endif
