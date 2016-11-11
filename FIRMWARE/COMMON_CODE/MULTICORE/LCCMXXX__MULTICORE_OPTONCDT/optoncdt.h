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


 #ifndef _OPTONCDT_H_
#define _OPTONCDT_H_
	#include <stdint.h>
	#include "uart.h"

		/*******************************************************************************
		Defines
		*******************************************************************************/
		#define OPTONCDT_HIGH_ERROR_VALUE 51
		#define OPTONCDT_LOW_ERROR_VALUE -1

		/** Set up flags */
		typedef enum
		{
			OPTONCDT_FLAG_NONE = 0,
			OPTONCDT_FLAG_NEW = 1,
			OPTONCDT_FLAG_ERROR = 2,
		}
		optoncdt_flag_t;


		/** State types for the OPTONCDT state machine */
		typedef enum
		{

			/** do nothing*/
			OPTONCDT_STATE__IDLE = 0U,

			/** We are in an error condition */
			OPTONCDT_STATE__ERROR,

			/** init the device, force a reset */
			OPTONCDT_STATE__INIT_DEVICE,

			/** Issue the conversion command*/
			OPTONCDT_STATE__BEGIN_SAMPLE,

			/** Wait for a number of processing loops to expire */
			OPTONCDT_STATE__WAIT_LOOPS,

			/** Read the ADC */
			OPTONCDT_STATE__PROCESS_SAMPLE,

			/** Compute the result */
			OPTONCDT_STATE__COMPUTE,


		}E_OPTONCDT_STATES_T;


		/*******************************************************************************
		Structures
		*******************************************************************************/

		struct _strOPTONCDT
		{
			uart_t* uart;
			Luint8 u8Buff[3];
			int id; // TODO: not used?

			Luint32 u32ValidResult;

			/** the current state */
			E_OPTONCDT_STATES_T eState;

			/** Keep track of bad values */
			Luint16 u16BadCount;

			/** Create flag type */
			optoncdt_flag_t flag;

			/** counter the number of main program loops */
			Luint32 u32LoopCounter;

			/** Last sampled ADC result*/
			Luint32 u32LastResult;

			/** The computed temp in deg C*/
			Lfloat32 f32Distance; // TODO: check type and size
				// formerly: float dist;

			/** TODO: What is this? not used */
			Lfloat* Offset;
		};



		/*******************************************************************************
		Function Prototypes
		*******************************************************************************/
		void optoncdt_laserPowerFull(laser_t* laser);
		void optoncdt_outputRs422(laser_t* laser);
		void optoncdt_poll(laser_t* laser);
		void optoncdt_structInit(laser_t* laser);



#endif