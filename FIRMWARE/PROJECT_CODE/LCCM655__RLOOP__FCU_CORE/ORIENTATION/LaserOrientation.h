/**
 * @file		LaserOrientation.h
 * @brief		Orientation and Position from Distance Lasers
 * @author		David
 * @copyright	rLoop Inc.
 * @st_fileID	
 */


/*******************************************************************************
Defines
*******************************************************************************/

/** State types for the TSYS01 state machine */
typedef enum
{

	/** do nothing*/
	LaserOrientation_STATE__IDLE = 0U,

	/** We are in an error condition */
	LaserOrientation_STATE__ERROR,

	/** init the device, force a reset */
	LaserOrientation_STATE__INIT_DEVICE,

	/** Read the constants from the device */
	LaserOrientation_STATE__READ_CONSTANTS,

	/** Waiting for the start of a conversion*/
	LaserOrientation_STATE__WAITING,

	/** Issue the conversion command*/
	LaserOrientation_STATE__BEGIN_SAMPLE,

	/** Wait for a number of processing loops to expire */
	LaserOrientation_STATE__WAIT_LOOPS,

	/** Read the ADC */
	LaserOrientation_STATE__RECALCULATE_ORIENTATION,

	/** Compute the result */
	LaserOrientation_STATE__COMPUTE,

}E_LaserOrientation_STATES_T;


/*******************************************************************************
Structures
*******************************************************************************/
struct _strComponent // TODO: might want to make a separate one for HE/laser, so that laser readings can be appended. or should hover height be set there similarly
{
	Lfloat32 f32Position[3]; // x,y,z
	Lfloat32 f32Measurement;
};


/*******************************************************************************
Function Prototypes
*******************************************************************************/

void vLaserOrientation__Init(void);
void vLaserOrientation__Process(void);
void vRecalcRoll(void);
void vRecalcPitch(void);
Lfloat32 f32PointToPlaneDistance(Lfloat32 f32Position[3]);
void vRecalcOrientation(void);
void vPrintPlane(void);
void vCalculateGroundPlane(struct sLaserA, struct sLaserB, struct sLaserC);

