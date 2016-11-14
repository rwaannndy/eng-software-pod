/**
 * @file		OPTONCDT.C
 * @brief		OPTONCDT Laser Distance Sensor
 * @author		Frank
 * @st_fileID	LCCMXXXR0.FILE.000
 */
/**
 * @addtogroup MULTICORE
 * @{ */
/**
 * @addtogroup OPTONCDT
 * @ingroup MULTICORE
 * @{ */
/**
 * @addtogroup OPTONCDT__CORE
 * @ingroup OPTONCDT
 * @{ */

// TODO: general todo notes
// - is a filter needed for this data?
// - additional error handling within the functions?
// - iron out how to handle failures of individual lasers
//		- if one laser has temporarily hung up, should still be able to pass on 
//		the good measurements.
// - confirm where all these data types should be initialized
// - How does addressing work with 6 lasers over uart?


#include "optoncdt.h" 

#define OPTONCDT__ERROR_VALUE 65467 // TODO: I believe this negates the need for checking range limits. will verify.
#define OPTONCDT__MIDDLE_VALUE (OPTONCDT__ERROR_VALUE / 2)
#define OPTONCDT__ERROR_COUNT_LIMIT 100

// Structure
struct _strOPTONCDT sOPTONCDT;

// Locals // TODO: I don't understand this distinction, why prototype here vs .h
void vOPTONCDT__ProcessSample(_strLaser* sLaser);


/***************************************************************************//**
 * @brief
 * Init any variables as needed
 * 
 * @st_funcMD5		XXXXXXXXXX
 * @st_funcID		YYYYYYYYYYY
 */
void vOPTONCDT__Init()
{
	// init vars for each laser
	vOPTONCDT_structInit(sLaserGround1);
	vOPTONCDT_structInit(sLaserGround2);
	vOPTONCDT_structInit(sLaserGround3);
	vOPTONCDT_structInit(sLaserBeam1);
	vOPTONCDT_structInit(sLaserBeam2);
	vOPTONCDT_structInit(sLaserBeam3);
	
	sOPTONCDT->eState = OPTONCDT_STATE__INIT_DEVICE;
}


/***************************************************************************//**
 * @brief
 * Main processing state machine.
 * 
 * @st_funcMD5		XXXXXXXXXXXX
 * @st_funcID		YYYYYYYYYY
 */

// TODO: due to quick measurement rate of optoncdt (<=2kHz), will probably be able to
//	scan all lasers in each loop, but need to verify this.
void vOPTONCDT__Process()
{
	switch(sOPTONCDT->eState)
	{
		case OPTONCDT_STATE__IDLE:
			// do nothing.
			break;

		case OPTONCDT_STATE__INIT_DEVICE:

			// init the devices and force resets
			vOPTONCDT__LaserPowerFull(_strLaser sLaserGround1);
			vOPTONCDT__LaserPowerFull(_strLaser sLaserGround2);
			vOPTONCDT__LaserPowerFull(_strLaser sLaserGround3);
			vOPTONCDT__LaserPowerFull(_strLaser sLaserBeam1);
			vOPTONCDT__LaserPowerFull(_strLaser sLaserBeam2);
			vOPTONCDT__LaserPowerFull(_strLaser sLaserBeam3);

			break;

		case OPTONCDT_STATE__WAIT_LOOPS:
			// TODO: I don't think we'll need this state
	
			// Wait for a number of processing loops to expire
			break;

		case OPTONCDT_STATE__BEGIN_SAMPLE:
			vOPTONCDT__poll(sOPTONCDT.sLaserGround1);
			vOPTONCDT__poll(sOPTONCDT.sLaserGround2);
			vOPTONCDT__poll(sOPTONCDT.sLaserGround3);
			vOPTONCDT__poll(sOPTONCDT.sLaserBeam1);
			vOPTONCDT__poll(sOPTONCDT.sLaserBeam2);
			vOPTONCDT__poll(sOPTONCDT.sLaserBeam3);

			break;
		
		case OPTONCDT_STATE__PROCESS_SAMPLE:
			vOPTONCDT__processSample(sOPTONCDT.sLaserGround1);
			vOPTONCDT__processSample(sOPTONCDT.sLaserGround2);
			vOPTONCDT__processSample(sOPTONCDT.sLaserGround3);
			vOPTONCDT__processSample(sOPTONCDT.sLaserBeam1);
			vOPTONCDT__processSample(sOPTONCDT.sLaserBeam2);
			vOPTONCDT__processSample(sOPTONCDT.sLaserBeam3);

			// sOPTONCDT State will be set within the above functions

			break;

		case OPTONCDT_STATE__COMPUTE:
			// Compute the result, converting from ADCbits to mm
			vOPTONCDT__UnitConversion(_strLaser sLaserGround1);
			vOPTONCDT__UnitConversion(_strLaser sLaserGround2);
			vOPTONCDT__UnitConversion(_strLaser sLaserGround3);
			vOPTONCDT__UnitConversion(_strLaser sLaserBeam1);
			vOPTONCDT__UnitConversion(_strLaser sLaserBeam2);
			vOPTONCDT__UnitConversion(_strLaser sLaserBeam3);

			break;

		case OPTONCDT_STATE__ERROR:
			/** We are in an error condition */
			break;
	}
}// end vOPTONCDT__Process()




/***************************************************************************//**
 * @brief
 * Init structure members
 * 
 * @param[in]		laser 		pointer to struct instance
 * @st_funcMD5		XXXXXXXXX
 * @st_funcID		YYYYYYYYY
 */
void vOPTONCDT__structInit(_strLaser* sLaser)
{
	sLaser->f32Distance = 0;
	sLaser->eFlag = OPTONCDT_FLAG_NONE;
	sLaser->u32LastResult = 0;
	sLaser->u16BadCount = 0;

	sLaser-> u32LastResult = OPTONCDT_ERROR_VALUE;


}


/***************************************************************************//**
 * @brief
 * Set the output of the laser to RS422
 * 
 * @param[in]		laser 		pointer to struct instance
 * @st_funcMD5		XXXXXXXXX
 * @st_funcID		YYYYYYYYY
 */
void vOPTONCDT__OutputRs422(_strLaser* sLaser)
{
	Luint8 u8Buff[32];
	int mlen; // TOOD: size?
	mlen = sprintf((char*)u8Buff, "OUTPUT RS422\r\n");
	sLaser->uart->writeBuff(u8Buff, mlen);
}


/***************************************************************************//**
 * @brief
 * Convert raw value read from laser to a distance in mm
 * 
 * @param[in]		laser   pointer to struct instance
 * @st_funcMD5		XXXXXXXXX
 * @st_funcID		YYYYYYYYY
 */
void vOPTONCDT__ProcessSample(_strLaser* sLaser)
{
	// assemble data packet according to datasheet
	Luint32 u32Result = sLaser->u8Buff[0] & 0x3F;
	Luint32 mid = sLaser->u8Buff[1] & 0x3F;
	Luint32 hi = sLaser->u8Buff[2] & 0x0F;

	u32Result += mid << 6;
	u32Result += hi << 12;

	sLaser->u32LastResult = u32Result;
	if (u32Result == OPTONCDT__ERROR_VALUE)
	{
		if (sLaser->u16BadCount > OPTONCDT__ERROR_COUNT_LIMIT)
		{
			if (sLaser->u32ValidResult > OPTONCDT__MIDDLE_VALUE) {
				sLaser->f32Distance = OPTONCDT__HIGH_ERROR_VALUE;
			}
			else {
				sLaser->f32Distance = OPTONCDT__LOW_ERROR_VALUE;
			}
			sLaser->eFlag = OPTONCDT__FLAG_ERROR;
			sLaser->eState = OPTONCDT_STATE__ERROR;
			sOPTONCDT->eState = OPTONCDT_STATE__ERROR;

		}
		else
		{
			sLaser->u16BadCount++;
			sLaser->eState = OPTONCDT_STATE__ERROR;
			sOPTONCDT->eState = OPTONCDT_STATE__ERROR;

		}
	}
	else
	{
		// Store the latest good value
		sLaser->u32ValidResult = u32Result; // unitless/bits
		// Use good value to compute final result
		sLaser->eState = OPTONCDT_STATE__COMPUTE;
	}
}


/***************************************************************************//**
 * @brief
 * Convert raw value read from laser to a distance in mm
 * 
 * @param[in]		laser   pointer to struct instance
 * @st_funcMD5		XXXXXXXXX
 * @st_funcID		YYYYYYYYY
 */
void vOPTONCDT__Poll(_strLaser* sLaser)
{
	while (sLaser->uart->avail() > 0)
	{
		Luint8 u8C = sLaser->uart->read();
		if ((u8C & (1 << 7)) != 0) // higher bits, sent last
		{
			sLaser->u8Buff[2] = u8C;
			sLaser->eState = OPTONCDT_STATE__PROCESS_SAMPLE;
		}
		else if ((u8C & 0xC0) != 0) // middle bits
		{
			sLaser->u8Buff[1] = u8C;
		}
		else if ((u8C & 0xC0) == 0) // lower bits
		{
			sLaser->u8Buff[0] = u8C;
		}
	}
}


/***************************************************************************//**
 * @brief
 * Convert raw value read from laser to a distance in mm
 * 
 * @param[in]		laser   pointer to struct instance
 * @st_funcMD5		XXXXXXXXX
 * @st_funcID		YYYYYYYYY
 */
void vOPTONCDT__UnitConversion(_strLaser* sLaser)
{
	// convert to millimeters using eqn from datasheet
	sLaser->f32Distance = (((Lfloat32) u32ValidResult * 102.0 / 65520.0) - 1.0) * 50.0 / 100.0; // TODO: syntax ok?

	sLaser->u16BadCount = 0; // clear consecutive error counter
	sLaser->eFlag = OPTONCDT__FLAG_NEW;

}


/***************************************************************************//**
 * @brief
 * Convert raw value read from laser to a distance in mm
 * 
 * @param[in]		laser   pointer to struct instance
 * @st_funcMD5		XXXXXXXXX
 * @st_funcID		YYYYYYYYY
 */
void vOPTONCDT__LaserPowerFull(_strLaser* sLaser)
{
	Luint8 u8Buff[32];
	Lint32 mlen; // TODO: check size
	mlen = sprintf((char*)u8Buff, "LASERPOW FULL\r\n");
	sLaser->uart->writeBuff(u8Buff, mlen);
}

