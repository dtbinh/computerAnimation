#include "precomp.h"
#include "particleManagerTinkerToy.h"
#include <iostream>

const float timeStep = 0.1;

ParticleManagerTinkerToy::ParticleManagerTinkerToy( std::vector <Particle*>& particles, std::vector<IForce*>& forces ) : ParticleManager( particles, forces )
{
	m_pSolver = new RK4Solver(timeStep);
	m_indexSpringParticle = -1;
	m_fGravityOn = true;
}

void ParticleManagerTinkerToy::performStep()
{
	Eigen::Matrix< double, Eigen::Dynamic, 1 > currentState( m_particles.size() * 6 );
	Eigen::Matrix< double, Eigen::Dynamic, 1 > newState( m_particles.size() * 6 );
	accumulateState( currentState );
	m_pSolver->evaluateNextState( currentState, newState, *this );
	disperseState( newState );
}

void ParticleManagerTinkerToy::performStep( double x, double y )
{
	if ( m_indexSpringParticle != -1 )
	{
		addSpringForce( x, y );
	}
	performStep();
}

void ParticleManagerTinkerToy::addParticle(Particle* pParticle, double x, double y)
{
	pParticle->mPosition[0] = x;
	pParticle->mPosition[1] = y;
	ParticleManager::addParticle( pParticle );
	std::cout<<"Particle position "<<x<<" "<<y<<" "<<m_particles[0]->mPosition;
}

int ParticleManagerTinkerToy::findClosestParticleIndex( double x, double y, bool fIgnoreLast )
{
	double distance =  DBL_MAX;
	int closestParticleIndex = -1;
	Eigen::Vector3d positionCurrent( x, y, 0 );
	int lastParticleIndex = fIgnoreLast ? m_particles.size() - 1: m_particles.size();
	for (int i = 0; i < lastParticleIndex; i++) //Ignore last added particle
	{
		Eigen::Vector3d distVector = m_particles[i]->mPosition - positionCurrent;
		double distCur = distVector.squaredNorm();
		if ( distCur < distance )
		{
			distance = distCur;
			closestParticleIndex = i;
		}
	}
	return closestParticleIndex;
}

void ParticleManagerTinkerToy::addConstraint( double x, double y, bool fBeadConstraint )
{
	FixedDistanceConstraint* pConstraint = NULL;
	if ( !fBeadConstraint )
	{
		int indexClosestParticle = findClosestParticleIndex( x, y, true );
		Eigen::Vector3d distVector = m_particles[indexClosestParticle]->mPosition - m_particles[m_particles.size() - 1]->mPosition;
		double distance = distVector.norm();
		pConstraint = new FixedDistanceConstraint( m_particles[indexClosestParticle], m_particles[m_particles.size() -1 ], indexClosestParticle, m_particles.size() - 1, distance );
		m_pConstraintManager->addConstraint( pConstraint );
	}
	else
	{
		Eigen::Vector3d positionCurrent( x, y, 0 );
		if ( abs(positionCurrent.norm() - 0.2) < 0.02 )
		{
			double distance = 0.2;
			pConstraint = new FixedDistanceConstraint( m_particles[m_particles.size() -1 ], NULL, m_particles.size() - 1, -1, distance );
			m_particles[m_particles.size() - 1]->mColor[0] = 0.0;
			m_particles[m_particles.size() - 1]->mColor[2] = 1.0;
			m_particles[m_particles.size() - 1]->mPosition.normalize();
			m_particles[m_particles.size() - 1]->mPosition *= 0.2;
			m_pConstraintManager->addConstraint( pConstraint );
		}
	}
}

void ParticleManagerTinkerToy::enableSpring( double x, double y )
{
	m_indexSpringParticle = findClosestParticleIndex( x, y );
}

void ParticleManagerTinkerToy::removeSpring()
{
	m_indexSpringParticle = -1;
	if (m_pForces.size() == 3)
	{
		IForce* pForce = m_pForces[2];
		m_pForces.pop_back();
		delete pForce;
	}
}

void ParticleManagerTinkerToy::addSpringForce( double x, double y )
{
	if (m_pForces.size() == 2)
	{
		m_pForces.push_back( new SpringForce( m_particles[m_indexSpringParticle], x, y ) );
	}
	else
	{
		IForce* pForce = m_pForces[2];
		m_pForces.pop_back();
		delete pForce;
		m_pForces.push_back( new SpringForce( m_particles[m_indexSpringParticle], x, y ) );
	}
}