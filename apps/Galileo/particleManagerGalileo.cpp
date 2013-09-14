#include "precomp.h"
#include "particleManagerGalileo.h"

const float timeStep = 0.01;

ParticleManagerGalileo::ParticleManagerGalileo( std::vector <Particle*>& particles, std::vector< IForce* >& forces ) : ParticleManager( particles, forces )
{
	m_time = 0;
	m_pSolver.push_back( new EulerSolver(timeStep) );
	m_pSolver.push_back( new MidPointSolver(timeStep) );
}

void ParticleManagerGalileo::performStep()
{
	Eigen::Matrix< double, Eigen::Dynamic, 1 > currentStateEuler( m_particles.size() * 6 );
	Eigen::Matrix< double, Eigen::Dynamic, 1 > newStateEuler( m_particles.size() * 6 );
	Eigen::Matrix< double, Eigen::Dynamic, 1 > currentStateMidPoint( m_particles.size() * 6 );
	Eigen::Matrix< double, Eigen::Dynamic, 1 > newStateMidPoint( m_particles.size() * 6 );

	accumulateState( currentStateEuler );
	m_pSolver[0]->evaluateNextState( currentStateEuler, newStateEuler, *this );

	accumulateState( currentStateMidPoint );
	m_pSolver[1]->evaluateNextState( currentStateMidPoint, newStateMidPoint, *this );

	//Reset the state to the beginning. These are hacks considering the nature of this question (different integrators for different things
	disperseState( currentStateEuler );

	Eigen::Matrix< double, Eigen::Dynamic, 1 > eulerState = newStateEuler.topRows(6);
	disperseState( eulerState, 0 );

	Eigen::Matrix< double, Eigen::Dynamic, 1 > midpointState = newStateMidPoint.middleRows<6>(6);
	disperseState( midpointState, 1 );

	m_time += timeStep;
	formulaEvaluateNextState( 2 );	
}

void ParticleManagerGalileo::formulaEvaluateNextState( int particleNum )
{
	static Eigen::Vector3d position = m_particles[particleNum]->mPosition;
	m_particles[particleNum]->mPosition = position + 0.5 * m_time * m_time * Eigen::Vector3d( 0, -0.1, 0 );
}

