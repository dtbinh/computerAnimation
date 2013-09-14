#include "MyWorld.h"
#include "Particle.h"
#include "math/UtilsMath.h"

using namespace Eigen;

MyWorld::MyWorld(int _numParticles): mParticleManager( mParticles, mForces ){
    // Create particles
    for (int i = 0; i < _numParticles; i++) {
        Particle *p = new Particle();
        mParticles.push_back(p);

        // Init particle positions randomly
        for (int j = 0; j < 3; j++) {
            double position = dart_math::random(-0.49, 0.49);
            mParticles[i]->mPosition[j] = position;
        }
		if ( mParticles[i]->mPosition.norm() >= 0.5 )
		{
			mParticles[i]->mPosition.normalize();
			mParticles[i]->mPosition /= 2;
		}
    }

    // Init cube position
    mCubePosition.setZero();
	mForces.push_back( new GravitationalForce( mParticles, Eigen::Vector3d( 0, -0.01, 0) ) );
	mForces.push_back( new WindForce( mParticles ) );
}

MyWorld::~MyWorld() {
    for (int i = 0; i < mParticles.size(); i++)
        delete mParticles[i];
	for (int i = 0; i < mForces.size(); i++)
		delete mForces[i];

	mParticles.clear();
}

void MyWorld::simulate() {
	mParticleManager.performStep();
}

void MyWorld::moveContainer( double x, double y )
{
	mCubePosition[0] += x;
	mCubePosition[1] += y;
	mParticleManager.setContainerCenter( mCubePosition );
}