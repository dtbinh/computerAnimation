#include "precomp.h"
#include "particleManagerSnowCube.h"
#include <iostream>

const float timeStep = 0.05;

ParticleManagerSnowCube::ParticleManagerSnowCube( std::vector <Particle*>& particles, std::vector<IForce*>& forces ) : ParticleManager( particles, forces ), m_containerCenter( 0, 0, 0 )
{
	m_pSolver = new EulerSolver(timeStep);
}

void ParticleManagerSnowCube::performStep()
{
	Eigen::Matrix< double, Eigen::Dynamic, 1 > currentState( m_particles.size() * 6 );
	Eigen::Matrix< double, Eigen::Dynamic, 1 > newState( m_particles.size() * 6 );
	accumulateState( currentState );
	m_pSolver->evaluateNextState( currentState, newState, *this );
	disperseState( newState );

	resolveCollision();
}

void ParticleManagerSnowCube::resolveCollision()
{
	for (int i = 0; i < m_particles.size(); i++)
	{
		if ( 0.5 - (m_particles[i]->mPosition - m_containerCenter).norm() < 0.01 )
		{
			Eigen::Vector3d unitNormal = -( m_particles[i]->mPosition - m_containerCenter );
			unitNormal.normalize();
			float dot = m_particles[i]->mVelocity.dot( unitNormal );
			if ( dot < 0 )
			{
				m_particles[i]->mVelocity = ( m_particles[i]->mVelocity - 2 * dot * unitNormal );
				m_particles[i]->mPosition = m_containerCenter - 0.49 * unitNormal;
			}
		}
	}
}

void ParticleManagerSnowCube::resolveCollisionMotion( const Eigen::Vector3d& deltaMotion )
{
	//Assume deltaMotion = velocity of wall
	for (int i = 0; i < m_particles.size(); i++)
	{
		if ( 0.5 - (m_particles[i]->mPosition - m_containerCenter).norm() < 0.01 )
		{
			Eigen::Vector3d unitNormal = -( m_particles[i]->mPosition - m_containerCenter );
			unitNormal.normalize();
			Eigen::Vector3d velocitySphere = deltaMotion.dot( unitNormal ) * unitNormal;
			float dot = m_particles[i]->mVelocity.dot( unitNormal );
			m_particles[i]->mVelocity = ( m_particles[i]->mVelocity - 2 * dot * unitNormal + 2 * velocitySphere );
			m_particles[i]->mPosition = m_containerCenter - 0.49 * unitNormal;
		}
	}
}

void ParticleManagerSnowCube::setContainerCenter( const Eigen::Vector3d& center )
{
	Eigen::Vector3d delta = center - m_containerCenter;
	m_containerCenter = center; 
	resolveCollisionMotion( delta );
}