#ifndef _SOLVER_H_
#define _SOLVER_H_

#include "Eigen\Eigen"
class ParticleManager;

class ISolver
{
public:
	virtual void evaluateNextState( const Eigen::Matrix<double, Eigen::Dynamic, 1>& currentState, Eigen::Matrix<double, Eigen::Dynamic, 1>& newState, ParticleManager& particleManager ) = 0;
	virtual ~ISolver() = 0 {}
};

class EulerSolver : public ISolver
{
private:
	float m_timeStep;
public:
	EulerSolver( float timeStep ) : m_timeStep( timeStep ) {}
	void evaluateNextState( const Eigen::Matrix<double, Eigen::Dynamic, 1>& currentState, Eigen::Matrix<double, Eigen::Dynamic, 1>& newState, ParticleManager& particleManager ) override;
};

class MidPointSolver : public ISolver
{
private:
	float m_timeStep;
public:
	MidPointSolver( float timeStep ) : m_timeStep( timeStep ) {}
	void evaluateNextState( const Eigen::Matrix<double, Eigen::Dynamic, 1>& currentState, Eigen::Matrix<double, Eigen::Dynamic, 1>& newState, ParticleManager& particleManager ) override;
};

class RK4Solver : public ISolver
{
private:
	float m_timeStep;
public:
	RK4Solver( float timeStep ) : m_timeStep( timeStep ) {}
	void evaluateNextState( const Eigen::Matrix<double, Eigen::Dynamic, 1>& currentState, Eigen::Matrix<double, Eigen::Dynamic, 1>& newState, ParticleManager& particleManager ) override;
};


#endif//_SOLVER_H_