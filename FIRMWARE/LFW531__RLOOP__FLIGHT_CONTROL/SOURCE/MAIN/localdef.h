#ifndef LOCALDEF_H_
#define LOCALDEF_H_

	//The PCB's main files
	#include <../../../BOARD_SUPPORT/lpcb235r0__board_support.h>



/*******************************************************************************
AMC7812
*******************************************************************************/
	#define C_LOCALDEF__LCCM658__ENABLE_THIS_MODULE							(1U)
	#if C_LOCALDEF__LCCM658__ENABLE_THIS_MODULE == 1U

		/** Num devices on the bus */
		#define C_LOCALDEF__LCCM658__NUM_DEVICES							(1U)

		/** Testing Options */
		#define C_LOCALDEF__LCCM658__ENABLE_TEST_SPEC						(0U)

		/** Main include file */
		#include <MULTICORE/LCCM658__MULTICORE__AMC7812/amc7812.h>
	#endif //#if C_LOCALDEF__LCCM658__ENABLE_THIS_MODULE == 1U


/*******************************************************************************
RLOOP - PI COMMUNICATIONS MODULE
*******************************************************************************/
	#define C_LOCALDEF__LCCM656__ENABLE_THIS_MODULE							(1U)
	#if C_LOCALDEF__LCCM656__ENABLE_THIS_MODULE == 1U

		//arch
		#define C_LOCALDEF__LCCM656__USE_ON_RM4								(1U)
		#define C_LOCALDEF__LCCM656__USE_ON_WIN32							(0U)

<<<<<<< HEAD
=======
		/** enable the receiver side? */
<<<<<<< HEAD
		#define C_LOCALDEF__LCCM656__ENABLE_RX								(0U)
>>>>>>> ca5677ac2f0b28ece0e72dd3dd6fa4dbd87a73f4
=======
		#define C_LOCALDEF__LCCM656__ENABLE_RX								(1U)
>>>>>>> 14f6ceb89e5be0c0a8e117e5b97ba00af63afff8

		/** Testing Options */
		#define C_LOCALDEF__LCCM656__ENABLE_TEST_SPEC						(0U)

		/** Main include file */
		#include <LCCM656__RLOOP__PI_COMMS/pi_comms.h>
	#endif //#if C_LOCALDEF__LCCM656__ENABLE_THIS_MODULE == 1U

/*******************************************************************************
RLOOP - FLIGHT CONTROL UNIT - CORE
*******************************************************************************/
	#define C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE							(1U)
	#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U

		/** Enable or disable the PiComms layer */
		#define C_LOCALDEF__LCCM655__ENABLE_PI_COMMS						(0U)


		#define C_LOCALDEF__LCCM655__EEPROM_OFFSET__BRAKES_HEADER			(40U)
		#define C_LOCALDEF__LCCM655__EEPROM_OFFSET__BRAKE0_ZERO				(41U)
		#define C_LOCALDEF__LCCM655__EEPROM_OFFSET__BRAKE0_SPAN				(42U)
		#define C_LOCALDEF__LCCM655__EEPROM_OFFSET__BRAKE1_ZERO				(43U)
		#define C_LOCALDEF__LCCM655__EEPROM_OFFSET__BRAKE1_SPAN				(44U)
		#define C_LOCALDEF__LCCM655__EEPROM_OFFSET__BRAKES_CRC				(45U)

		#define C_LOCALDEF__LCCM655__EEPROM_OFFSET__STEP0_VELOC				(46U)
		#define C_LOCALDEF__LCCM655__EEPROM_OFFSET__STEP0_ACCEL				(47U)
		#define C_LOCALDEF__LCCM655__EEPROM_OFFSET__STEP1_VELOC				(48U)
		#define C_LOCALDEF__LCCM655__EEPROM_OFFSET__STEP1_ACCEL				(49U)
		#define C_LOCALDEF__LCCM655__EEPROM_OFFSET__STEP_CRC				(50U)

		/** ADC Sample Limits */
		#define C_LOCALDEF__LCCM655__ADC_SAMPLE__LOWER_BOUND				(300U)
		#define C_LOCALDEF__LCCM655__ADC_SAMPLE__UPPER_BOUND				(3000U)

		/** Testing Options */
		#define C_LOCALDEF__LCCM655__ENABLE_TEST_SPEC						(0U)

		/** Main include file */
		#include <LCCM655__RLOOP__FCU_CORE/fcu_core.h>
	#endif //#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U


#endif /* LOCALDEF_H_ */
