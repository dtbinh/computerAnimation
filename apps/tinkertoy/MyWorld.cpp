#include "MyWorld.h"
#include "Particle.h"
#include "math/UtilsMath.h"

using namespace Eigen;

MyWorld::MyWorld(int _numParticles) : mParticleManager( mParticles, mForces ), mConstraintManager( mParticleManager ) {
    // Create particles
    for (int i = 0; i < _numParticles; i++) {
        Particle *p = new Particle();
        mParticles.push_back(p);
    }
    // Init particle position
    mParticles[0]->mPosition[0] = 0.2;
	FixedDistanceConstraint* pBeadOnWireConstraint = new FixedDistanceConstraint( mParticles[0], NULL, 0, -1, 0.2 );
	mConstraintManager.addConstraint( pBeadOnWireConstraint );
	mForces.push_back( new GravitationalForce( mParticles ) );
	mForces.push_back( new DampingForce( mParticles, 0.01 ) );
	mFPauseSimulation = false;
	mFSpringMode = false;
	mFShowSpring = false;
}

MyWorld::~MyWorld() {
    for (int i = 0; i < mParticles.size(); i++)
        delete mParticles[i];
    mParticles.clear();
}

void MyWorld::simulate( double x, double y ) {
	if (!mFPauseSimulation)
		mParticleManager.performStep( x, y );
}

void MyWorld::onRightMouseClick(double x, double y)
{
	if ( m_newParticle.get() != NULL )
	{
		mParticleManager.addConstraint( x, y );
	}
}


void MyWorld::onLeftMouseClick(double x, double y)
{
	if (mFSpringMode)
	{
		mFShowSpring = true;
		mParticleManager.enableSpring( x, y );
	}
	else
	{
		mFPauseSimulation = true;
		m_newParticle.release();
		m_newParticle.reset( new Particle() );
		mParticleManager.addParticle( m_newParticle.get(), x, y );
		mParticleManager.addConstraint( x, y, true );
	}
}

void MyWorld::resumeSimulation()
{
	m_newParticle.release();
	mFPauseSimulation = false;
}

void MyWorld::draw(renderer::RenderInterface* pRI) 
{
    // Draw particles
    for (int i = 0; i < getNumParticles(); i++)
        getParticle(i)->draw(pRI);

	if ( mFShowSpring && mForces.size() >= 3 )
	{
		dynamic_cast<SpringForce*>(mForces[2])->draw();
	}

	mConstraintManager.drawConstraints();
}
