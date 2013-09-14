#include "constraintManager.h"
#include "particleManager.h"
#include <iostream>

const double k_ks = 0.1;
const double k_kd = 0.1;

ConstraintManager::ConstraintManager( ParticleManager& particleManager ) : m_particleManager( particleManager ), m_ks( k_ks ), m_kd( k_kd )
{
	m_particleManager.setConstraintManager( this );
}

ConstraintManager::~ConstraintManager()
{
	for (int i = 0; i < m_constraintForces.size(); i++)
	{
		delete (m_constraintForces[i]);
	}
	m_constraintForces.clear();
}

void ConstraintManager::resizeJacobian()
{
	m_jacobian.resize( m_constraintForces.size(), m_particleManager.getNumParticles() * 3);
	m_jacobianDeriv.resize( m_constraintForces.size(), m_particleManager.getNumParticles() * 3);

	m_constraint.resize( m_constraintForces.size() );
	m_constraintDeriv.resize( m_constraintForces.size() );
}

void ConstraintManager::onParticleAdded()
{
	resizeJacobian();
}

void ConstraintManager::addConstraint( FixedDistanceConstraint* pConstraint )
{
	pConstraint->setConstraintIndex( m_constraintForces.size() );
	m_constraintForces.push_back( pConstraint );
	resizeJacobian();
}

void ConstraintManager::computeConstraintForces()
{
	for (std::vector< FixedDistanceConstraint* >::iterator iter = m_constraintForces.begin(); iter < m_constraintForces.end(); ++iter)
	{
		(*iter)->populateConstraintMatrix( m_constraint );
		(*iter)->populateJacobian( m_jacobian );
		(*iter)->populateJacobianDerivative( m_jacobianDeriv );
	}
	Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > invMassMatrix;
	Eigen::Matrix< double, Eigen::Dynamic, 1 > velocityCurrent;
	Eigen::Matrix< double, Eigen::Dynamic, 1 > forceCurrent;
	m_particleManager.getInvMassMatrix( invMassMatrix );
	m_particleManager.getCurrentState( velocityCurrent, false, true );
	m_particleManager.getCurrentDerivative( forceCurrent, false, true );
	m_constraintDeriv = m_jacobian * velocityCurrent;
	Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > jacobianInvJacobianT = m_jacobian * invMassMatrix * m_jacobian.transpose();
	Eigen::Matrix< double, Eigen::Dynamic, 1 > jacobianDerivVelocity = m_jacobianDeriv * velocityCurrent;
	Eigen::Matrix< double, Eigen::Dynamic, 1 > jacobianInvMassForce = m_jacobian * invMassMatrix * forceCurrent;
	Eigen::Matrix< double, Eigen::Dynamic, 1 > b = -jacobianDerivVelocity - jacobianInvMassForce - m_ks*m_constraint - m_kd*m_constraintDeriv;
	if (jacobianInvJacobianT.determinant() != 0)
	{
		m_lagrangeMultipliers = jacobianInvJacobianT.ldlt().solve( b );
	}
	else
	{
		m_lagrangeMultipliers.resize( m_constraintForces.size() );
		m_lagrangeMultipliers.setZero();
	}
}

void ConstraintManager::applyConstraintForces()
{
	Eigen::Matrix< double, Eigen::Dynamic, 1 > constraintForces = m_jacobian.transpose() * m_lagrangeMultipliers;
	m_particleManager.applyConstraintForces( constraintForces );
}

void ConstraintManager::drawConstraints()
{
	for ( std::vector< FixedDistanceConstraint* >::iterator iter = m_constraintForces.begin(); iter != m_constraintForces.end(); ++iter )
	{
		(*iter)->draw();
	}
}