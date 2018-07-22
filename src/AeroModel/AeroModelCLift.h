/**
 * @brief Aerodynamic Model Lift Coefficient.
 *
 * Computation of the lift coefficient.
 * 
 */
#ifndef __AEROMODELCLIFT_H_INCLUDED__
#define __AEROMODELCLIFT_H_INCLUDED__

#include "interp.h"
#include "AeroModelCoeff.h"


class CAeroModelCoeffLift : public CAeroModelCoeff
{
	// Constructor/Destructor
	public:
		CAeroModelCoeffLift();

	// Operations
	public:
		/**
		 * Compute the lift coefficient.
		 */
		virtual BOOL compute();

	// Implementation
	protected:
		FLOAT	m_fClBa;			//!< Lift based on Dynamics
		FLOAT	m_fClElev;			//!< Lift as a result of the elevator
		FLOAT	m_fClAth;			//!< Lift as a result of asymetric thrust
		FLOAT	m_fClGe;			//!< Lift due to ground effect
		FLOAT	m_fClFF;			//!< Lift due a flap malfunction
		FLOAT	m_fClIce;			//!< Lift degradation due to ice buildup
		FLOAT	m_fClBias;			//!< Constant for data matching purposes

	// Data Tables
	private:
		// Icing Effects
		static FLOAT	s_axisIceAlphaDeg[];
		static FLOAT	s_resultIce[];
		static LT1D		s_tableIce;
		
		// Basic Lift
		static FLOAT	s_axisBaAlpha[];
		static FLOAT	s_axisBaTcx[];
		static FLOAT	s_axisBaFlap[];
		static FLOAT	s_resultBa[];
		static LT3D		s_tableBa;
};

#endif	// __AEROMODELCLIFT_H_INCLUDED__