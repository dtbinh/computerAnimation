#include "precomp.h"
#include "solver.h"
#include "particleManager.h"
#include <iostream>

void EulerSolver::evaluateNextState( const Eigen::Matrix<double, Eigen::Dynamic, 1>& currentState, Eigen::Matrix<double, Eigen::Dynamic, 1>& newState, ParticleManager& particleManager )
{
	Eigen::Matrix< double, Eigen::Dynamic, 1 > derivative;
	particleManager.derivativeEvaluate( derivative );
	newState = currentState + m_timeStep * derivative;
}

void MidPointSolver::evaluateNextState( const Eigen::Matrix<double, Eigen::Dynamic, 1>& currentState, Eigen::Matrix<double, Eigen::Dynamic, 1>& newState, ParticleManager& particleManager )
{
	Eigen::Matrix< double, Eigen::Dynamic, 1 > derivative;
	particleManager.derivativeEvaluate( derivative );

	Eigen::Matrix<double, Eigen::Dynamic, 1> midPointValue = currentState + ( ( m_timeStep / 2 ) * derivative );
	particleManager.disperseState( midPointValue );

	particleManager.derivativeEvaluate( derivative );
	newState = currentState + m_timeStep * derivative;
}

void RK4Solver::evaluateNextState( const Eigen::Matrix<double, Eigen::Dynamic, 1>& currentState, Eigen::Matrix<double, Eigen::Dynamic, 1>& newState, ParticleManager& particleManager )
{
	Eigen::Matrix< double, Eigen::Dynamic, 1 > derivative;
	particleManager.derivativeEvaluate( derivative );
	Eigen::Matrix<double, Eigen::Dynamic, 1> k1 = m_timeStep * derivative;
	
	Eigen::Matrix<double, Eigen::Dynamic, 1> value1 = currentState + ( ( m_timeStep / 2 ) * derivative );
	particleManager.disperseState( value1 );
	particleManager.derivativeEvaluate( derivative );
	Eigen::Matrix<double, Eigen::Dynamic, 1> k2 = m_timeStep * derivative;
	
	Eigen::Matrix<double, Eigen::Dynamic, 1> value2 = currentState + ( ( m_timeStep / 2 ) * derivative );
	particleManager.disperseState( value2 );
	particleManager.derivativeEvaluate( derivative );
	Eigen::Matrix<double, Eigen::Dynamic, 1> k3 = m_timeStep * derivative;

	Eigen::Matrix<double, Eigen::Dynamic, 1> value3 = currentState + m_timeStep * derivative;
	particleManager.disperseState( value3 );
	particleManager.derivativeEvaluate( derivative );
	Eigen::Matrix<double, Eigen::Dynamic, 1> k4 = m_timeStep * derivative;

	newState = currentState + k1/6 + k2/3 + k3/3 + k4/6;
}