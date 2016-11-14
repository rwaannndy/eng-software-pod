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
	#include "uart.h" // TODO: not needed

		/*******************************************************************************
		Defines
		*******************************************************************************/
		#define OPTONCDT__HIGH_ERROR_VALUE 51
		#define OPTONCDT__LOW_ERROR_VALUE -1

		/** Set up flags */
		typedef enum
		{
			OPTONCDT__FLAG_NONE = 0,
			OPTONCDT__FLAG_NEW = 1,
			OPTONCDT__FLAG_ERROR = 2,
		}E_OPTONCDT_FLAG;


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
			// substruct to incorporate 6 lasers
				// TODO: might want this moved out of here..
			struct _strLaser
			{
				/** The computed temp, in deg C*/
				Lfloat32 f32Distance;

				/** Latest good value, in bits */
				Luint32 u32ValidResult;

				/** Latest sampled ADC result, in bits*/
				Luint32 u32LastResult;

				/** Counter of consecutive bad values */
				Luint16 u16BadCount;

				/** Create flag type */
				E_OPTONCDT_FLAG eFlag;

				/** Counter for the number of main program loops */
				Luint32 u32LoopCounter; // TODO: tie in to STATE__WAIT_LOOPS if it's needed

				/** Track the state of each individual laser */
				E_OPTONCDT_STATES_T eState;

				// TODO: fix these...
				uart_t* uart;
				Luint8 u8Buff[3];
				// Luint8 u8ID; // TODO: not used?
				// Lfloat32* f32Offset; // TODO: what is this? not used.

			};

			struct _strLaser sLaserGround1;
			struct _strLaser sLaserGround2;
			struct _strLaser sLaserGround3;
			struct _strLaser sLaserBeam1;
			struct _strLaser sLaserBeam2;
			struct _strLaser sLaserBeam3;


			/** the current state */
			// use the states of each individual laser to determine the overall state
			//		e.g. one laser is in the error state, overall state must be error
			E_OPTONCDT_STATES_T eState;

		};



		/*******************************************************************************
		Function Prototypes
		*******************************************************************************/
		void vOPTONCDT__LaserPowerFull(_strLaser* sLaser);
		void vOPTONCDT__OutputRs422(_strLaser* sLaser);
		void vOPTONCDT__Poll(_strLaser* sLaser);
		void vOPTONCDT__structInit(_strLaser* sLaser);


#endif