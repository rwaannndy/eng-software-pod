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


#include "optoncdt.h" 

#define OPTONCDT_ERROR_VALUE 65467
#define OPTONCDT_MIDDLE_VALUE (OPTONCDT_ERROR_VALUE / 2)
#define OPTONCDT_ERROR_COUNT_LIMIT 100
void vOPTONCDT_processSample(_strOPTONCDT* laser);

void vOPTONCDT__Init()
{

	// init struct
	void vOPTONCDT_structInit(_strOPTONCDT* laser)
	{
		laser->f32Distance = 0;
		laser->flag = 0;
		laser->u32LastResult = 0;
		laser->u16BadCount = 0;
		laser->eState = OPTONCDT_STATE__INIT_DEVICE;

	}

}

// TODO: due to quick measurement rate of optoncdt (<=2kHz), will probably be able to
//	scan all lasers in each loop, verify this..
void vOPTONCDT__Process()
{
	switch(laser.eState)

			/** do nothing*/
		case OPTONCDT_STATE__IDLE:
			// do nothing.
			break;


		case OPTONCDT_STATE__INIT_DEVICE:

			// run this function for each laser
			// init the device, force a reset
			void vOPTONCDT_laserPowerFull(_strOPTONCDT* laser)
			{
				Luint8 buff[32];
				int mlen;
				mlen = sprintf((char*)buff, "LASERPOW FULL\r\n");
				laser->uart->writeBuff(buff, mlen);
			}

			break;


		case OPTONCDT_STATE__BEGIN_SAMPLE:

			// run this function for each laser
			void vOPTONCDT_poll(_strOPTONCDT* laser)
			{
				while (laser->uart->avail() > 0)
				{
					Luint8 c = laser->uart->read();
					if ((c & (1 << 7)) != 0) // higher bits, sent last
					{
						laser->buff[2] = c;
						laser.eState = OPTONCDT_STATE__PROCESS_SAMPLE;
					}
					else if ((c & 0xC0) != 0) // middle bits2
					{
						laser->buff[1] = c;
					}
					else if ((c & 0xC0) == 0) // lower bits
					{
						laser->buff[0] = c;
					}
				}
			}
			break;


		case OPTONCDT_STATE__WAIT_LOOPS:
			// Wait for a number of processing loops to expire
			break;

		
		case OPTONCDT_STATE__PROCESS_SAMPLE:

			break;


		case OPTONCDT_STATE__COMPUTE:
			// Compute the result

			// convert to millimeters using eqn from datasheet
			Lfloat32 laser->f32Distance = ((u32LastResult * 102.0 / 65520.0) - 1.0) * 50.0 / 100.0;

			laser->u32ValidResult = result;
			laser->u16BadCount = 0; // clear error
			laser->flag = OPTONCDT_FLAG_NEW;

			break;


		case OPTONCDT_STATE__ERROR:
			/** We are in an error condition */
			break;
}



void vOPTONCDT_outputRs422(_strOPTONCDT* laser)
{
	Luint8 buff[32];
	int mlen;
	mlen = sprintf((char*)buff, "OUTPUT RS422\r\n");
	laser->uart->writeBuff(buff, mlen);
}


void vOPTONCDT_processSample(_strOPTONCDT* laser)
{
	// assemble data packet according to datasheet
	Luint32 result = buff[0] & 0x3F;
	Luint32 mid = buff[1] & 0x3F;
	Luint32 hi = buff[2] & 0x0F;
	result += mid << 6;
	result += hi << 12;

	laser->u32LastResult = result;
	if (result == OPTONCDT_ERROR_VALUE)
	{
		if (laser->u16BadCount > OPTONCDT_ERROR_COUNT_LIMIT)
		{
			if (laser->u32ValidResult > OPTONCDT_MIDDLE_VALUE) {
				laser->f32Distance = OPTONCDT_HIGH_ERROR_VALUE;
			}
			else {
				laser->f32Distance = OPTONCDT_LOW_ERROR_VALUE;
			}
			laser->flag = OPTONCDT_FLAG_ERROR;
			laser.eState = OPTONCDT_STATE__ERROR;
		}
		else
		{
			laser->u16BadCount++;
			laser.eState = OPTONCDT_STATE__ERROR;
		}
	}
	else
	{
		// Good value received, use it to compute final result
		laser.eState = OPTONCDT_STATE__COMPUTE;
	}
}


