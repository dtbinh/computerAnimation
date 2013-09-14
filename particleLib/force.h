#ifndef _FORCE_H_
#define _FORCE_H_

#include <vector>
#include "Particle.h"

const Eigen::Vector3d k_gravity(0, -0.01, 0);

class IForce
{
public:
	virtual void calculate() = 0;
	virtual ~IForce() = 0 {}
	bool fEnabled() { return m_fEnabled; }
	void enable( bool fEnable ) { m_fEnabled = fEnable; }
protected:
	bool m_fEnabled;
};

class GravitationalForce : public IForce
{
private:
	std::vector< Particle* >& m_particles;
	Eigen::Vector3d m_gravity;
public:
	GravitationalForce( std::vector< Particle* >& particles, const Eigen::Vector3d& gravity = k_gravity ) : m_particles( particles ), m_gravity( gravity ) {}
	void calculate() override;
};

class WindForce : public IForce
{
private:
	std::vector< Particle*> &m_particles;
public:
	WindForce( std::vector< Particle* >& particles ) : m_particles( particles ) {}
	void calculate() override;
};

class SpringForce : public IForce
{
private:
	Particle* m_pParticle;
	Eigen::Vector3d m_positionOther;
public:
	SpringForce( Particle* pParticle, double x, double y ) : m_pParticle( pParticle ), m_positionOther( x, y, 0 ) {}
	void calculate() override;
	void draw();
};

class DampingForce : public IForce
{
private:
	std::vector<Particle*>& m_pParticles;
	double m_dampingConstant;
public:
	DampingForce( std::vector<Particle*>& pParticles, double dampConstant ) : m_pParticles( pParticles ), m_dampingConstant( dampConstant ) {}
	void calculate() override;
};

class ConstraintForce : public IForce
{
public:
	virtual void populateJacobian( Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& jacobian ) = 0;
	virtual void populateJacobianDerivative( Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& jacobianDerivative ) = 0;
	virtual void populateConstraintMatrix( Eigen::Matrix< double, Eigen::Dynamic, 1 >& constraintMatrix ) = 0;
	void setConstraintIndex( int constraintIndex ) { m_constraintIndex = constraintIndex; }
protected:
	int m_constraintIndex;
};

class FixedDistanceConstraint : public ConstraintForce
{
private:
	Particle *m_pP1,*m_pP2;
	int m_p1Index, m_p2Index;
	Eigen::Vector3d m_forceVector;
	double m_distance;

public:
	FixedDistanceConstraint( Particle* pP1, Particle* pP2, int p1Index, int p2Index, double distance ) : m_pP1(pP1), m_pP2(pP2), m_p1Index(p1Index), m_p2Index(p2Index), m_distance( distance ) {}
	void populateJacobian( Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& jacobian ) override;
	void populateJacobianDerivative( Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& jacobianDerivative ) override;
	void populateConstraintMatrix( Eigen::Matrix< double, Eigen::Dynamic, 1 >& constraintMatrix ) override;
	void calculate() override;
	void setForceVector( const Eigen::Vector3d& forceVector );
	void draw();
};

#endif//_FORCE_H_