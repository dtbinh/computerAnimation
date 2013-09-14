#ifndef _PARTICLEMANAGER_TIKERTOY_H_
#define _PARTICLEMANAGER_TIKERTOY_H_

#include "particleManager.h"

class ParticleManagerTinkerToy : public ParticleManager
{
public:
	ParticleManagerTinkerToy( std::vector <Particle*>& particles, std::vector< IForce* >& forces );
	void performStep() override;
	void performStep( double x, double y );
	void addParticle( Particle* pParticle, double x, double y );
	void addConstraint( double x, double y, bool fBeadConstraint = false );
	void enableSpring( double x, double y );
	void removeSpring();

private:
	int ParticleManagerTinkerToy::findClosestParticleIndex( double x, double y, bool fIgnoreLast = false );
	void addSpringForce( double x, double y );

	int m_indexSpringParticle;
	int m_springForceIndex;
	bool m_fGravityOn;
	ISolver* m_pSolver;
};

#endif//_PARTICLEMANAGER_TIKERTOY_H_