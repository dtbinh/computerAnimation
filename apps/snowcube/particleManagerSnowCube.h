#ifndef _PARTICLEMANAGER_SNOWCUBE_H_
#define _PARTICLEMANAGER_SNOWCUBE_H_

#include "particleManager.h"

class ParticleManagerSnowCube : public ParticleManager
{
public:
	ParticleManagerSnowCube( std::vector <Particle*>& particles, std::vector<IForce*>& forces );
	void performStep() override;
	void setContainerCenter( const Eigen::Vector3d& center );

private:
	void ParticleManagerSnowCube::resolveCollision();
	void ParticleManagerSnowCube::resolveCollisionMotion( const Eigen::Vector3d& deltaMotion );
	
	ISolver* m_pSolver;
	Eigen::Vector3d m_containerCenter;
};

#endif//_PARTICLEMANAGER_SNOWCUBE_H_