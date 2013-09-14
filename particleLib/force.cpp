#include "precomp.h"
#include "force.h"
#include "GL\glut.h"

const double k_spring = 0.1;
const double m_wind = 0.01;

void GravitationalForce::calculate()
{
	if ( m_fEnabled )
	{
		for (std::vector<Particle*>::iterator iter = m_particles.begin(); iter != m_particles.end(); ++iter)
		{
			(*iter)->mAccumulatedForce += m_gravity;
		}
	}
}

void WindForce::calculate()
{
	if ( m_fEnabled )
	{
		for (std::vector<Particle*>::iterator iter = m_particles.begin(); iter != m_particles.end(); ++iter)
		{
			int randomX = rand()%10 - 5;
			int randomZ = rand()%10 - 5;
			Eigen::Vector3d wind( randomX * m_wind, 0, randomZ * m_wind );
			(*iter)->mAccumulatedForce += wind;
		}
	}
}

void DampingForce::calculate()
{
	if ( m_fEnabled )
	{
		for (std::vector<Particle*>::iterator iter = m_pParticles.begin(); iter != m_pParticles.end(); ++iter)
		{
			(*iter)->mAccumulatedForce += -m_dampingConstant * (*iter)->mVelocity;
		}
	}
}

void SpringForce::calculate()
{
	if ( m_fEnabled )
	{
		m_pParticle->mAccumulatedForce -= k_spring * (m_pParticle->mPosition - m_positionOther);
	}
}

void SpringForce::draw()
{
	glDisable(GL_LIGHTING);
	glColor4d(0.0, 1.0, 0.0, 1.0);
	glLineWidth(3.0f);
	glBegin( GL_LINE_LOOP );
	glVertex3d( m_pParticle->mPosition[0], m_pParticle->mPosition[1], m_pParticle->mPosition[2] );
	glVertex3d( m_positionOther[0], m_positionOther[1], m_positionOther[2] );
	glVertex3d( m_pParticle->mPosition[0], m_pParticle->mPosition[1], m_pParticle->mPosition[2] );
	glEnd();
	glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void FixedDistanceConstraint::populateConstraintMatrix( Eigen::Matrix< double, Eigen::Dynamic, 1 >& constraintMatrix )
{
	constraintMatrix.row(m_constraintIndex).setZero();
	if ( m_pP2 == NULL )
	{
		constraintMatrix.row(m_constraintIndex).array()[0] = m_pP1->mPosition.norm() - m_distance;
	}
	else
	{
		Eigen::Vector3d distVector = m_pP1->mPosition - m_pP2->mPosition;
		constraintMatrix.row(m_constraintIndex).array()[0] = distVector.norm() - m_distance;
	}
}

void FixedDistanceConstraint::populateJacobian( Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >& jacobian )
{
	jacobian.row(m_constraintIndex).setZero();
	for (int i = 0; i < 3; i++)
	{
		if ( m_pP2 != NULL )
		{
			jacobian.row(m_constraintIndex).array()[3*m_p1Index + i] = m_pP1->mPosition[i] - m_pP2->mPosition[i];
			jacobian.row(m_constraintIndex).array()[3*m_p2Index + i] = m_pP2->mPosition[i] - m_pP1->mPosition[i];
		}
		else
		{
			jacobian.row(m_constraintIndex).array()[3*m_p1Index + i] = m_pP1->mPosition[i];
		}
	}
}

void FixedDistanceConstraint::populateJacobianDerivative( Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic>& jacobianDerivative )
{
	jacobianDerivative.row(m_constraintIndex).setZero();
	for (int i = 0; i < 3; i++)
	{
		if ( m_pP2 != NULL )
		{
			jacobianDerivative.row(m_constraintIndex).array()[3*m_p1Index + i] = m_pP1->mVelocity[i] - m_pP2->mVelocity[i];
			jacobianDerivative.row(m_constraintIndex).array()[3*m_p2Index + i] = m_pP2->mVelocity[i] - m_pP1->mVelocity[i];
		}
		else
		{
			jacobianDerivative.row(m_constraintIndex).array()[3*m_p1Index + i] = m_pP1->mVelocity[i];
		}
	}
}

void FixedDistanceConstraint::setForceVector( const Eigen::Vector3d& forceVector )
{
	m_forceVector = forceVector;
}

void FixedDistanceConstraint::calculate()
{
	assert( "Should not be called");
}

void FixedDistanceConstraint::draw()
{
	glDisable(GL_LIGHTING);
	glColor4d(1.0, 0.0, 0.0, 1.0);
	glLineWidth(3.0f);
	if ( m_pP2 != NULL )
	{
		glBegin( GL_LINE_LOOP );
		glVertex3d( m_pP1->mPosition[0], m_pP1->mPosition[1], m_pP1->mPosition[2] );
		glVertex3d( m_pP2->mPosition[0], m_pP2->mPosition[1], m_pP2->mPosition[2] );
		glVertex3d( m_pP1->mPosition[0], m_pP1->mPosition[1], m_pP1->mPosition[2] );
		glEnd();
	}
	glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}