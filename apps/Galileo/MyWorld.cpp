#include "MyWorld.h"
#include "Particle.h"

using namespace Eigen;

MyWorld::MyWorld(int _numParticles) : mParticleManager( mParticles, mForces ){
    // Create particles
    for (int i = 0; i < _numParticles; i++) {
        Particle *p = new Particle();
        mParticles.push_back(p);
    }

    // Init particle positions (default is 0, 0, 0)
    mParticles[0]->mPosition[0] = -0.3;
    mParticles[0]->mPosition[1] = 20.0;
    mParticles[1]->mPosition[1] = 20.0;
    mParticles[2]->mPosition[0] = 0.3;
    mParticles[2]->mPosition[1] = 20.0;

    // Init particle colors (default is red)
    mParticles[0]->mColor = Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
    mParticles[1]->mColor = Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
	
	mForces.push_back( new GravitationalForce( mParticles, Eigen::Vector3d( 0, -0.1, 0 ) ) );
}

MyWorld::~MyWorld() {
    for (int i = 0; i < mParticles.size(); i++)
        delete mParticles[i];
  
	for (int i = 0; i < mForces.size(); i++)
		delete mForces[i];

	mParticles.clear();
}

void MyWorld::simulate() {
	
	static Eigen::Vector3d position = mParticles[2]->mPosition;
	if ( position[1] - mParticles[2]->mPosition[1] <= 30 )
	{
		mParticleManager.performStep();
	}
}
