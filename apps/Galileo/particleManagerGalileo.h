#ifndef _PARTICLEMANAGER_GALILEO_H_
#define _PARTICLEMANAGER_GALILEO_H_

#include "particleManager.h"

class ParticleManagerGalileo : public ParticleManager
{
public:
	ParticleManagerGalileo( std::vector <Particle*>& particles, std::vector< IForce* >& forces );
	void performStep() override;

private:
	void formulaEvaluateNextState( int particleNum );

	std::vector<ISolver*> m_pSolver;
	float m_time;
};

#endif//_PARTICLEMANAGER_GALILEO_H_