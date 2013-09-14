#ifndef _CONSTRAINTMANAGER_H_
#define _CONSTRAINTMANAGER_H_

#include <Eigen\Eigen>
#include <vector>
#include "force.h"
class ParticleManager;

class ConstraintManager
{
public:
	ConstraintManager( ParticleManager& particleManager );
	~ConstraintManager();

	void addConstraint( FixedDistanceConstraint* pConstraint );
	void computeConstraintForces();
	void applyConstraintForces();
	void removeConstraint();
	void drawConstraints();
	void onParticleAdded();

private:
	void resizeJacobian();
	
	ParticleManager& m_particleManager;
	std::vector< FixedDistanceConstraint* > m_constraintForces; //owned
	Eigen::Matrix< double, Eigen::Dynamic, 1 > m_lagrangeMultipliers;
	Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > m_jacobian;
	Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > m_jacobianDeriv;
	Eigen::Matrix< double, Eigen::Dynamic, 1 > m_constraint;
	Eigen::Matrix< double, Eigen::Dynamic, 1 > m_constraintDeriv;

	double m_ks;
	double m_kd;
};

#endif//_CONSTRAINTMANAGER_H_
