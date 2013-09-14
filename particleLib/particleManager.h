#ifndef _PARTICLE_MANAGER_H_
#define _PARTICLE_MANAGER_H_

#include "Particle.h"
#include "solver.h"
#include "force.h"
#include "constraintManager.h"

class ParticleManager
{
public:
	ParticleManager( std::vector <Particle*>& particles, std::vector<IForce*>& forces );
	void addParticle( Particle* pParticle );
	void setConstraintManager( ConstraintManager* pConstraintManager ) { m_pConstraintManager = pConstraintManager; }

	//Interface for each problem
	virtual void performStep() = 0;
	void applyConstraintForces( const Eigen::Matrix< double, Eigen::Dynamic, 1 >& constraintForces );

	//State query functions
	void getCurrentState( Eigen::Matrix< double, Eigen::Dynamic, 1>& currentState, bool fPosition, bool fVelocity );
	void getCurrentDerivative( Eigen::Matrix< double, Eigen::Dynamic, 1>& currentState, bool fPosition, bool fVelocity );
	void getInvMassMatrix( Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& invMassMatrix );
	int getNumParticles() { return m_particles.size(); }

	//Interface with the Solver
	void derivativeEvaluate( Eigen::Matrix< double, Eigen::Dynamic, 1 >& derivative);
	void disperseState(Eigen::Matrix< double, Eigen::Dynamic, 1 >& newState, int particleNum = -1);

protected:
	//The particleNum parameter is just here to allow for code sharing with Galileo. This should not be here.
	void accumulateState(Eigen::Matrix< double, Eigen::Dynamic, 1 >& currentState, int particleNum = -1);
	void accumulateDerivative(Eigen::Matrix< double, Eigen::Dynamic, 1 >& derivative, int particleNum = -1);

	std::vector<Particle*>& m_particles;
	std::vector<IForce*>& m_pForces;
	ConstraintManager* m_pConstraintManager;

private:
	void accumulateStateForParticle(int particleNum, Eigen::Matrix< double, Eigen::Dynamic, 1 >& currentState, bool fPosition, bool fVelocity, bool fSingleParticle);
	void accumulateDerivativeForParticle(int particleNum, Eigen::Matrix< double, Eigen::Dynamic, 1 >& derivative, bool fVelocity, bool fAcceleration, bool fSingleParticle);
	void disperseStateForParticle(int particleNum, Eigen::Matrix< double, Eigen::Dynamic, 1 >& newState, bool fSingleParticle);
	void clearForceAccumulators();
};

#endif//_PARTICLE_MANAGER_H_