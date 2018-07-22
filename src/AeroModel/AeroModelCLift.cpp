/* ============================================================================
 *	    Copyright (c) 
 * ============================================================================
 *
 *~	FILENAME:			AeroModelCLift.cpp
 *
 *~	CSC NUMBER:			not_entered
 *
 *~	TITLE:				Aerodynamic Model Lift Coefficient
 *
 *~	DESCRIPTION:		This file contains the implementation of the
 *						Aerodynamic Model Lift Coefficient class.
 *
 *~	NOTES:				none
 *
 *~	ORIGINAL AUTHOR:	S Garrood
 *
 *~	CREATION DATE:		2006-01-13
 *
 *~	CHANGE HISTORY:
 *
 *	$Log:   M:/Archives/CFTS/C90B/Software/Src/Sim/AeroModel/AeroModelCLift.cpv  $
*
*   Rev 1.1   Aug 31 2006 13:25:16   ifitzgerald
*DR633: Bruce's pre dry run update. 
 *
 * ============================================================================
 */

#include <C90Defs.h>
#include <Sim_Defs.h>
#include "Interp.h"					// Lookup table interpolation routines
#include "AeroModelCLift.h"			// The Aeromodel Interface

// Icing Effects

AAC_FLOAT	CAeroModelCoeffLift::s_axisIceAlphaDeg[] =
	{
		0.0f,  4.00f,  8.00f, 10.00f, 12.00f
	};

AAC_FLOAT	CAeroModelCoeffLift::s_resultIce[] =
	{
		0.0f, -0.03f, -0.21f, -0.37f, -0.39f
	};

AAC_LT1D	CAeroModelCoeffLift::s_tableIce =
	{
		sizeof( s_axisIceAlphaDeg ) / sizeof(AAC_FLOAT),
		s_axisIceAlphaDeg,
		s_resultIce
	};



// Basic Lift

AAC_FLOAT	CAeroModelCoeffLift::s_axisBaAlpha[] =
	{ 
		-8.0f, -4.0f, 0.0f, 4.0f, 8.0f, 10.0f, 12.0f, 14.0f, 16.0f, 20.0f 
	};

AAC_FLOAT	CAeroModelCoeffLift::s_axisBaTcx[] = 
	{  
		0.0f, 0.1f, 0.2f, 0.6f 
	};

AAC_FLOAT	CAeroModelCoeffLift::s_axisBaFlap[] = 
	{  
		0.0f, 100.0f 
	};

AAC_FLOAT	CAeroModelCoeffLift::s_resultBa[] =
	{
		-0.52f, -0.08f, 0.35f, 0.70f, 1.06f, 1.14f, 1.20f, 1.21f, 1.12f, 1.04f, 
		-0.49f, -0.04f, 0.40f, 0.76f, 1.13f, 1.27f, 1.38f, 1.39f, 1.34f, 1.24f, 
		-0.47f, -0.03f, 0.42f, 0.80f, 1.19f, 1.35f, 1.47f, 1.48f, 1.44f, 1.33f, 
		-0.46f,  0.0f,  0.44f, 0.86f, 1.26f, 1.44f, 1.58f, 1.62f, 1.60f, 1.50f,
		
		 0.07f,  0.46f, 0.85f, 1.24f, 1.50f, 1.55f, 1.53f, 1.40f, 1.22f, 1.05f, 
		 0.14f,  0.54f, 0.95f, 1.34f, 1.60f, 1.66f, 1.67f, 1.54f, 1.38f, 1.24f, 
		 0.17f,  0.60f, 1.02f, 1.42f, 1.71f, 1.77f, 1.80f, 1.70f, 1.57f, 1.38f, 
		 0.32f,  0.78f, 1.23f, 1.62f, 1.93f, 1.99f, 2.02f, 1.96f, 1.84f, 1.61f
	};

AAC_LT3D	CAeroModelCoeffLift::s_tableBa = 
	{
		sizeof(s_axisBaAlpha)/sizeof(AAC_FLOAT),	// num 'X' points
		sizeof(s_axisBaTcx)/sizeof(AAC_FLOAT),		// num 'Y' points
		sizeof(s_axisBaFlap)/sizeof(AAC_FLOAT),		// num 'Z' points
		s_axisBaAlpha,
		s_axisBaTcx,
		s_axisBaFlap,
		s_resultBa
	};




//-----------------------------------------------------------------------------
//
//~	NAME:				CAeroModelCoeffLift::CAeroModelCoeffLift
//
//~	CSU_NUMBER:			not_entered
//
//~	TITLE:				Aerodynamic Model Lift Coefficient Class Constructor
//
//~	DESCRIPTION:		Constructs the Lift Coefficient object.
//
//~	PARAMETERS:			none
//
//~	NOTES:				none
//
//~ REFERENCES:			none
//
//~	ORIGINAL AUTHOR:	Stuart Garrood
//
//~	CREATION DATE:		2006-01-13
//
//~	CHANGE HISTORY:
//
//  MODIFIED:	BY:		REASON:
//  ----------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
CAeroModelCoeffLift::CAeroModelCoeffLift()	:
	CAeroModelCoeff()
{
	m_fClBa			= 0;			// Basic body coefficient of lift	
	m_fClDyn		= 0;			// lift based on Dynamics
	m_fClElev		= 0;			// lift as a result of the elevator
	m_fClAth		= 0;			// lift as a result of asymetric thrust
	m_fClGe			= 0;			// lift due to ground effect
	m_fClFF			= 0;			// lift due a flap malfunction
	m_fClIce		= 0;			// lift degradation due to ice buildup
	m_fClBias		= 0;			// Constant for data matching purposes
}


//-----------------------------------------------------------------------------
//
//~	NAME:				CAeroModelCoeffLift::Compute()
//
//~	CSU_NUMBER:			not_entered
//
//~	TITLE:				Lift Coefficient Computor
//
//~	DESCRIPTION:		Compute aerodynamic lift coefficient.
//
//~	PARAMETERS:			none
//
//~	NOTES:				none
//
//~ REFERENCES:			none
//
//~	ORIGINAL AUTHOR:	Stuart Garrood
//
//~	CREATION DATE:		2006-01-13
//
//~	CHANGE HISTORY:
//
//  MODIFIED:	BY:		REASON:
//  ----------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
AAC_BOOL	
CAeroModelCoeffLift::Compute()
{
	AAC_BOOL bValid = FALSE;


	//==== Basic rigid-airplane lift coefficient ===========
	//              90E1624 AS, pp. A-1, A-6, A-7           

	// Calculate basic lift coefficient based on the angle of attack (deg),
	// symmetric thrust coefficient and the flap deflection (%), 
	// Ref: 90E1624 AS, pg. A-1, A-6, A-7
	
	m_fClBa =	Interp3D(s_tableBa, 
					 (AAC_FLOAT)s_pData->dAlphaB_d, 
					 s_pCoeffData->fTcx, 
					 s_pData->fDeltaF_pct, 
					 FALSE, 
					 FALSE, 
					 FALSE);
	
	// Incremental lift due to airplane dynamics
	// Ref: 90E1624 AS, pg. A-1

	m_fClDyn = (AAC_FLOAT)( ( (C_CLAD * s_pData->dAlphaDot_rps) + 
						    (C_CLQ * s_pData->dQs_rps) ) * s_pCoeffData->fCHat );


	// Incremental lift due to elevator deflection from 0 deg
	// Ref: 90E1624 AS, pg. A-2

	//--	Need to first get Cm due to elevator; this will be used again
	//--	in the Cm calculation.										

	m_fClElev = -s_pCoeffData->fCmElev * C_XCOLH;


	// Incremental lift due to assymetric thrust
	// Ref: 90E1624 AS, pg. A-2

	m_fClAth = (AAC_FLOAT)((( C_CLATO + C_CLATA * s_pData->dAlphaB_d ) +
		        C_CLATF * s_pData->fDeltaF_pct / 100.0f ) * ( ABS( s_pCoeffData->fTcd ) / 0.4 ));


	// Incremental lift due to ground effect
	// Ref: 90E1624 AS, pg. A-3

	AAC_FLOAT fGE  = (AAC_FLOAT)( MAX( 0.0, 1.0 - ( 2.0 * s_pCoeffData->fHGear / C_BWREF )));
	m_fClGe = C_CLGEO * fGE;


	// Incremental lift due to flap failures
	// Ref: 90E1624 AS, pg. A-3

	AAC_FLOAT fldfo = ( s_pData->fDfavg_pct - s_pData->fDeltaF_pct ) * 0.04f;
	AAC_FLOAT clff1 = (AAC_FLOAT)( C_CLFFO + ( C_CLFFA * s_pData->dAlphaB_d ));
	m_fClFF  = clff1 * fldfo;


	// Lift degradation due to ice buildup
	// Ref: 90E1624 AS, pp. A-4, A-8

	AAC_FLOAT clIce = Interp1D(s_tableIce, 
							   (AAC_FLOAT)s_pData->dAlphaB_d);
	m_fClIce  = clIce * s_pData->fKIce;


	//======== Sum lift coefficients to C_lift  ============

	m_fCoeff = m_fClBa				// basic rigid body lift
				+ m_fClDyn			// dynamic lift
				+ m_fClElev			// elevator
				+ m_fClAth			// asymmetric thrust
				+ m_fClGe			// ground effect
				+ m_fClFF    		// flap failure
				+ m_fClIce    		// ice buildup
				+ m_fClBias;		// bias

	s_pCoeffData->fClStar = m_fClBa + m_fClDyn;

	s_pData->fCLift = m_fCoeff;

	bValid = TRUE;

	return bValid;
}
