#include "precomp.h"
#include "particleManager.h"
#include "Eigen\Eigen"
#include <iostream>

ParticleManager::ParticleManager( std::vector <Particle*>& particles, std::vector <IForce*>& pForces ) : m_particles( particles ), m_pForces( pForces )
{
	m_pConstraintManager = NULL;
}

void ParticleManager::addParticle( Particle* pParticle )
{
	m_particles.push_back( pParticle );
	if ( m_pConstraintManager != NULL )
	{
		m_pConstraintManager->onParticleAdded();
	}
}

//State query functions
void ParticleManager::getCurrentState( Eigen::Matrix< double, Eigen::Dynamic, 1 >& currentState, bool fPosition, bool fVelocity )
{
	int stateSize = (fPosition & fVelocity) ? 6 : 3;
	currentState.resize( m_particles.size() * stateSize );
	for (int i = 0; i < m_particles.size(); i++)
	{
		accumulateStateForParticle( i, currentState, fPosition, fVelocity, false/*fSingleParticle*/ );
	}
}

void ParticleManager::getCurrentDerivative( Eigen::Matrix< double, Eigen::Dynamic, 1 >& currentDerivative, bool fVelocity, bool fAcceleration )
{
	int stateSize = (fAcceleration & fVelocity) ? 6 : 3;
	currentDerivative.resize( m_particles.size() * stateSize );
	for (int i = 0; i < m_particles.size(); i++)
	{
		accumulateDerivativeForParticle( i, currentDerivative, fVelocity, fAcceleration, false/*fSingleParticle*/ );
	}
}

//TODO msati: Optimize this portion
void ParticleManager::getInvMassMatrix( Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& invMassMatrix )
{
	invMassMatrix.resize( m_particles.size() * 3, m_particles.size() * 3 );
	invMassMatrix.setZero();
	for (int i = 0; i < m_particles.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			invMassMatrix.row(3*i+j).array()[3*i+j] = 1/(m_particles[i]->mMass);
		}
	}
}

void ParticleManager::clearForceAccumulators()
{
	for (std::vector<Particle*>::iterator iter = m_particles.begin(); iter != m_particles.end(); ++iter )
	{
		(*iter)->mAccumulatedForce.setZero();
	}
}

void ParticleManager::derivativeEvaluate( Eigen::Matrix< double, Eigen::Dynamic, 1 >& derivative )
{
	clearForceAccumulators();
	for (std::vector<IForce*>::iterator iter = m_pForces.begin(); iter != m_pForces.end(); iter++)
	{
		(*iter)->calculate();
	}
	if ( m_pConstraintManager != NULL )
	{
		m_pConstraintManager->computeConstraintForces();
		m_pConstraintManager->applyConstraintForces();
	}

	derivative.resize( m_particles.size() * 6 );
	accumulateDerivative( derivative );
}

void ParticleManager::accumulateState(Eigen::Matrix< double, Eigen::Dynamic, 1 >& currentState, int particleNum)
{
	if (particleNum != -1)
	{
		accumulateStateForParticle( particleNum, currentState, true, true, true );
	}
	else
	{
		getCurrentState( currentState, true, true );
	}
}

void ParticleManager::accumulateDerivative(Eigen::Matrix< double, Eigen::Dynamic, 1 >& derivative, int particleNum)
{
	if (particleNum != -1)
	{
		accumulateDerivativeForParticle( particleNum, derivative, true, true, true);
	}
	else
	{
		getCurrentDerivative( derivative, true, true );
	}

}

void ParticleManager::disperseState(Eigen::Matrix< double, Eigen::Dynamic, 1 >& newState, int particleNum )
{
	if (particleNum != -1)
	{
		disperseStateForParticle( particleNum, newState, true );
	}
	else
	{
		for (int i = 0; i < m_particles.size(); i++)
		{
			disperseStateForParticle( i, newState, false );
		}
	}
}

void ParticleManager::accumulateStateForParticle(int particleNum, Eigen::Matrix< double, Eigen::Dynamic, 1>& currentState, bool fPosition, bool fVelocity, bool fSingleParticle)
{
	assert( fPosition == true || fVelocity == true ); 
	int stateSize = (fPosition & fVelocity) ? 6 : 3;
	int currentRow = fSingleParticle? 0 : particleNum * stateSize;
	if ( fPosition )
	{
		for (int i = 0; i < m_particles[particleNum]->mPosition.rows(); i++)
		{
			currentState.row(currentRow++) = m_particles[particleNum]->mPosition.row(i);
		}
	}
	if ( fVelocity )
	{
		for (int i = 0; i < m_particles[particleNum]->mVelocity.rows(); i++)
		{
			currentState.row(currentRow++) = m_particles[particleNum]->mVelocity.row(i);
		}
	}
}

void ParticleManager::accumulateDerivativeForParticle(int particleNum, Eigen::Matrix< double, Eigen::Dynamic, 1 >& derivative, bool fVelocity, bool fAcceleration, bool fSingleParticle)
{
	assert( fVelocity == true || fAcceleration == true ); 
	int stateSize = (fVelocity & fAcceleration) ? 6 : 3;
	int currentRow = fSingleParticle? 0 : particleNum * stateSize;
	if ( fVelocity )
	{
		for (int i = 0; i < m_particles[particleNum]->mVelocity.rows(); i++)
		{
			derivative.row(currentRow++) = m_particles[particleNum]->mVelocity.row(i);
		}
	}
	if ( fAcceleration )
	{
		for (int i = 0; i < m_particles[particleNum]->mAccumulatedForce.rows(); i++)
		{
			derivative.row(currentRow++) = (m_particles[particleNum]->mAccumulatedForce.row(i)/m_particles[particleNum]->mMass);
		}
	}
}

void ParticleManager::disperseStateForParticle(int particleNum, Eigen::Matrix< double, Eigen::Dynamic, 1 >& newState, bool fSingleParticle)
{
	int currentRow = fSingleParticle? 0 : particleNum * 6;
	for (int i = 0; i < m_particles[particleNum]->mPosition.rows(); i++)
	{
		m_particles[particleNum]->mPosition.row(i) = newState.row(currentRow++);
	}
	for (int i = 0; i < m_particles[particleNum]->mVelocity.rows(); i++)
	{
		m_particles[particleNum]->mVelocity.row(i) = newState.row(currentRow++);
	}
}

void ParticleManager::applyConstraintForces( const Eigen::Matrix< double, Eigen::Dynamic, 1 >& constraintForces )
{
	assert (constraintForces.size() == m_particles.size() * 3);
	int constraintRow = 0;
	for (int i = 0; i < m_particles.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if ( abs(constraintForces[constraintRow] >= 2) )
			{
				constraintRow++; //Hack - do nothing
				continue;
			}
			m_particles[i]->mAccumulatedForce[j] += constraintForces[constraintRow];
			constraintRow++;
		}
	}
}
