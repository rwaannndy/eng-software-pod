/* TODO:
 * doxygen compatible commenting
 */

 /**
 * @file		LaserOrientation.c
 * @brief		Orientation and Position from Distance Lasers
 * @author		David, acaratel
 * @copyright	rLoop Inc.
 * @st_fileID
 */

// TODO: If we are using anything involving floating point trig and that trig
  // is safety critical (i.e. pod distance or braking) then we must also do a
  // parallel equation in another data type to prevent the sorts of errors that
  // are commonly seen with floating point trig.


// TODO:
// - we now have a 4th laser facing the ground, which will help determine if
//	the frame of the pod has twisted. (change in flux 11.13.16)
// - will want to calc roll/pitch as seen by two sets of 3 lasers (a,b,c; b,c,d)
//	and return angle of twisting based on any discrepancy between the two measurements
// - check flags from optoncdt.c to verify that all lasers are functional.
//		- if one has failed, fall back to just pitch/roll with the 3 remaining

// acos func?



#include "LaserOrientation.h"
#include "optoncdt.c" // TODO: is this ok?

#define PI 3.14159265359
#define X 0U
#define Y 1U
#define Z 2U
#define A 0U
#define B 1U
#define C 2U
#define D 3U


//All units in mm
//The math doesn't care as long as you're consistent

//For the laser positions Z should be the reading when
//the HDK is sitting flat on the 4 hover engines

// Yaw value expressed accordingly the rloop system variable
// http://confluence.rloop.org/display/SD/System+Variables

// set laser structs
struct _strComponent sGroundLaser1;
struct _strComponent sGroundLaser2;
struct _strComponent sGroundLaser3;
struct _strComponent sGroundLaser4;

// set i-beam laser structs
struct _strComponent sBeamLaser1;
struct _strComponent sBeamLaser2;

// set hover engine structs
struct _strComponent sHE1;
struct _strComponent sHE2;
struct _strComponent sHE3;
struct _strComponent sHE4;
struct _strComponent sHE5;
struct _strComponent sHE6;
struct _strComponent sHE7;
struct _strComponent sHE8;

// init PodOrientation struct
struct _strPodOrientation sOrient;

// coordinate system: +x in direction of travel; +z up wrt gravity
void vLaserOrientation__Init(void)
{
	// TODO: All positions of components need their positions measured and assigned here.
		// blocked by installation of the components

	//Ground Facing Laser Positions
	sGroundLaser1.f32Position[3] = {8, 185, 35}; // ground laser 1 position {x,y,z}
	sGroundLaser2.f32Position[3] = {-112, 18, 35}; // ground laser 2 position {x,y,z}
	sGroundLaser3.f32Position[3] = {121, -53, 35}; // ground laser 3 position {x,y,z}
	sGroundLaser4.f32Position[3] = {}; // ground laser 4 position {x,y,z}

	// I-Beam laser positions
	sBeamLaser1.f32Position[3] = {25, 0, 35};  // i-beam laser 1 position {x,y,z}
	sBeamLaser2.f32Position[3] = {25, 100, 35};  //i-beam laser 2 position {x,y,z}

	//Hover Engine Positions {x,y,z} (from top view)
	sHE1.f32Position[3] = {61, 130, 0}; // Top Left {x,y,z}
	sHE2.f32Position[3] = {62, 129, 0}; // Top Right {x,y,z}
	sHE3.f32Position[3] = {62, 126, 0}; // Bottom Right {x,y,z}
	sHE4.f32Position[3] = {60, 128, 0}; // Bottom Left {x,y,z}

	// TODO: These engines are not yet implemented in this code.
	sHE5.f32Position[3] = {0, 0, 0}; // Top Left {x,y,z}
	sHE6.f32Position[3] = {0, 0, 0}; // Top Right {x,y,z}
	sHE7.f32Position[3] = {0, 0, 0}; // Bottom Right {x,y,z}
	sHE8.f32Position[3] = {0, 0, 0}; // Bottom Left {x,y,z}

	// Init PodOrientation values
	sOrient.s16Roll = 0;
	sOrient.s16Pitch = 0;
	sOrient.s16Yaw = 0;
	sOrient.f32Lateral = 0;
	sOrient.f32PlaneCoeffs[4] = {0,0,0,0};

	sOrient.eState = LaserOrientation_STATE__INIT;

	//vPrintPlane();
}

//Recalculate the orientation and engine heights
void vLaserOrientation__Process(void)
{
	//handle the state machine
	switch(sOrient.eState)
	{
		case LaserOrientation_STATE__IDLE:
			//do nothing
			break;

		case LaserOrientation_STATE__INIT:
			break;

		case LaserOrientation_STATE__RECALCULATE_PITCH_ROLL_TWIST:
			// check laser states in optoncdt.c
			// count which lasers are not in the error state and append them to array.
			Luint8 u8OperationalCount = 0U;
			Luint8 u8OperationalLasers[4]; // TODO: array of struct types? {sLaserGround1, sLaserGround2, ...}

			if (sLaserGround1.eState != OPTONCDT_STATE__ERROR)
			{
				u8OperationalLasers[u8OperationalCount] = 1U; // Laser 1 works, store a 1 in the array to denote this
				u8OperationalCount += 1U; // increment count of operational lasers
			}
			else
			{
				// laser 1 bad
			}
			if (sLaserGround2.eState == OPTONCDT_STATE__ERROR)
			{
				u8OperationalLasers[u8OperationalCount] = 2U; // Laser 2 works, store a 2 in the array to denote this
				u8OperationalCount += 1U; // increment count of operational lasers
			}
			else
			{
				// laser 2 bad
			}
			if (sLaserGround3.eState == OPTONCDT_STATE__ERROR)
			{
				u8OperationalLasers[u8OperationalCount] = 3U; // Laser 3 works, store a 3 in the array to denote this
				u8OperationalCount += 1U; // increment count of operational lasers
			}
			else
			{
				// laser 3 bad
			}
			if (sLaserGround4.eState == OPTONCDT_STATE__ERROR)
			{
				u8OperationalLasers[u8OperationalCount] = 4U; // Laser 4 works, store a 4 in the array to denote this
				u8OperationalCount += 1U; // increment count of operational lasers
			}
			else
			{
				// laser 4 bad
			}

			if (u8OperationalCount == 4U)
			{
				// calculate pitch, roll, and twist of the pod
			    vCalculateGroundPlane(sGroundLaser1, sGroundLaser2, sGroundLaser3); // TODO: make fns work together
			    vCalculateGroundPlane(sGroundLaser2, sGroundLaser3, sGroundLaser4);

				sHE1.f32Measurement = f32PointToPlaneDistance(sHE1.f32Position);
				sHE2.f32Measurement = f32PointToPlaneDistance(sHE2.f32Position);
				sHE3.f32Measurement = f32PointToPlaneDistance(sHE3.f32Position);
				sHE4.f32Measurement = f32PointToPlaneDistance(sHE4.f32Position);

				sHE5.f32Measurement = f32PointToPlaneDistance(sHE5.f32Position);
				sHE6.f32Measurement = f32PointToPlaneDistance(sHE6.f32Position);
				sHE7.f32Measurement = f32PointToPlaneDistance(sHE7.f32Position);
				sHE8.f32Measurement = f32PointToPlaneDistance(sHE8.f32Position);

				vRecalcPitch();
				vRecalcRoll();

				vCalculateTwist(); // TODO: write fn
			}
			else if (u8OperationalCount == 3U)
			{ 
				// calculate pitch and roll. cannot calculate twist.
			    vCalculateGroundPlane(u8OperationalLasers[0], u8OperationalLasers[1], u8OperationalLasers[2]); // TODO: THIS DOES NOT WORK YET!

				sHE1.f32Measurement = f32PointToPlaneDistance(sHE1.f32Position);
				sHE2.f32Measurement = f32PointToPlaneDistance(sHE2.f32Position);
				sHE3.f32Measurement = f32PointToPlaneDistance(sHE3.f32Position);
				sHE4.f32Measurement = f32PointToPlaneDistance(sHE4.f32Position);

				sHE5.f32Measurement = f32PointToPlaneDistance(sHE5.f32Position);
				sHE6.f32Measurement = f32PointToPlaneDistance(sHE6.f32Position);
				sHE7.f32Measurement = f32PointToPlaneDistance(sHE7.f32Position);
				sHE8.f32Measurement = f32PointToPlaneDistance(sHE8.f32Position);

				vRecalcPitch();
				vRecalcRoll();
			}
			else
			{
				// there are 2 or fewer operable lasers; can't compute twist/pitch/roll/HE heights.

			}
			sOrient.eState = LaserOrientation_STATE__RECALCULATE_YAW_AND_LATERAL;
			break;

		case LaserOrientation_STATE__RECALCULATE_YAW_AND_LATERAL:
			// check laser states in optoncdt.c
			// count which lasers are not in the error state and append them to array.
			Luint8 u8OperationalCount = 0U;
			Luint8 u8OperationalLasers[2];

			if (sLaserBeam1.eState == OPTONCDT_STATE__ERROR)
			{
				u8OperationalLasers[u8OperationalCount] = 1U; // Laser 1 works, store a 1 in the array to denote this
				u8OperationalCount += 1U; // increment count of operational lasers
			}
			else if (sLaserBeam2.eState == OPTONCDT_STATE__ERROR)
			{
				u8OperationalLasers[u8OperationalCount] = 2U; // Laser 2 works, store a 2 in the array to denote this
				u8OperationalCount += 1U; // increment count of operational lasers
			}
			else
			{
				// All lasers operational.
			}
			if (u8OperationalCount == 2U)
			{
				vRecalcYaw();
				vRecalcLateral();
			}
			else
			{
				// At least one i-beam laser is down; can't compute yaw/translation.
			}
			// sOrient.eState = ; // TODO: finish state flow
			break;

		case LaserOrientation_STATE__WAIT_LOOPS:
			// TODO: is this needed?
			break;

		case LaserOrientation_STATE__ERROR:
			//some error has happened
			break;

	}
}

Lfloat32 f32PointToPlaneDistance(Lfloat32 f32Position[3])
{
	return 
	(
		(f32PlaneCoeffs[A] * f32Position[X] + f32PlaneCoeffs[B] * f32Position[Y] + f32PlaneCoeffs[C] * f32Position[Z] + f32PlaneCoeffs[D]) / 
		sqrt((double)(f32PlaneCoeffs[A] * f32PlaneCoeffs[A] + f32PlaneCoeffs[B] * f32PlaneCoeffs[B] + f32PlaneCoeffs[C] * f32PlaneCoeffs[C]))
	);
}


//The angle between two planes that yields the roll
void vRecalcRoll(void)
{
	//Normal vector of the other plane
	Lfloat32 f32vec1x = 1, f32vec1y = 0, f32vec1z = 0;

	//Angle between two planes // TODO: Need to find a Lachlan func for this
	sOrient.s16Roll = (Lint16)(acos((double)((f32vec1x * f32PlaneCoeffs[A] + f32vec1y * f32PlaneCoeffs[B] + f32vec1z * f32PlaneCoeffs[C]) / sqrt((double)(f32PlaneCoeffs[A] * f32PlaneCoeffs[A] + f32PlaneCoeffs[B] * f32PlaneCoeffs[B] + f32PlaneCoeffs[C] * f32PlaneCoeffs[C])))) * 10000);
}

//The angle between two planes that yields the pitch
void vRecalcPitch(void)
{
	//Normal vector of the other plane
	Lfloat32 f32vec1x = 0, f32vec1y = 1, f32vec1z = 0;

	//Angle between two planes // TODO: Need to find a Lachlan func for this
	sOrient.s16Pitch = (Lint16)(acos((double)((f32vec1x * f32PlaneCoeffs[A] + f32vec1y * f32PlaneCoeffs[B] + f32vec1z * f32PlaneCoeffs[C]) / sqrt((double)(f32PlaneCoeffs[A] * f32PlaneCoeffs[A] + f32PlaneCoeffs[B] * f32PlaneCoeffs[B] + f32PlaneCoeffs[C] * f32PlaneCoeffs[C])))) * 10000);
}

void vPrintPlane(void)
{
	// printf("A:%f B:%f C:%f D:%f\n", f32PlaneCoeffs[0], f32PlaneCoeffs[1], f32PlaneCoeffs[2], f32PlaneCoeffs[3]);
}

//Calculate the ground plane given three points
//Ax + By + Cz + D = 0
void vCalculateGroundPlane(struct sLaserA, struct sLaserB, struct sLaserC)
{
	Lfloat32 f32Vec1X, f32Vec1Y, f32Vec1Z;
	Lfloat32 f32f32Vec2X, f32f32Vec2Y, f32f32Vec2Z;
	Lfloat32 f32f32XProductX, f32f32XProductY, f32f32XProductZ;

	//Calculate two vectors in the plane
	f32Vec1X = sGroundLaser1.f32Position[X] - sGroundLaser2.f32Position[X];
	f32Vec1Y = sGroundLaser1.f32Position[Y] - sGroundLaser2.f32Position[Y];
	f32Vec1Z = (sGroundLaser1.f32Position[Z] - sGroundLaser1.f32Measurement) - (sGroundLaser2.f32Position[Z] - sGroundLaser2.f32Measurement);
	f32Vec2X = sGroundLaser2.f32Position[X] - sGroundLaser3.f32Position[X];
	f32Vec2Y = sGroundLaser2.f32Position[Y] - sGroundLaser3.f32Position[Y];
	f32Vec2Z = (sGroundLaser2.f32Position[Z] - sGroundLaser2.f32Measurement) - (sGroundLaser3.f32Position[Z] - sGroundLaser3.f32Measurement);

	//Calculate the cross product of the vectors
	//to get a vector normal to the plane
	f32XProductX = f32Vec1Y*f32Vec2Z - f32Vec1Z*f32Vec2Y;
	f32XProductY = f32Vec1Z*f32Vec2X - f32Vec1X*f32Vec2Z;
	f32XProductZ = f32Vec1X*f32Vec2Y - f32Vec1Y*f32Vec2X;

	//The normal vector should be pointed in the +Z direction
	//It affects which side of the plane has negative distances
	if (f32XProductZ < 0){
		f32XProductX *= -1;
		f32XProductY *= -1;
		f32XProductZ *= -1;
	}

	//Plane in 3D: Ax + By + Cz + D = 0
	//A, B, C is the vector normal to the plane
	//Use one of our original points to calculate D
	f32PlaneCoeffs[A] = f32XProductX;
	f32PlaneCoeffs[B] = f32XProductY;
	f32PlaneCoeffs[C] = f32XProductZ;

	f32PlaneCoeffs[D] = -1 * (f32PlaneCoeffs[A] * sGroundLaser1.f32Position[X] + f32PlaneCoeffs[B] * sGroundLaser1.f32Position[Y] + f32PlaneCoeffs[C] * sGroundLaser1.f32Position[Z]);

}


void vRecalcYaw(void)
{
  Lfloat32 f32SDif = (Lfloat32)(sBeamLaser1.f32Measurement - sBeamLaser2.f32Measurement);
  Lfloat32 f32DTan = f32SDif / ((Lfloat32)(sBeamLaser1.f32Position[Z] - sBeamLaser2.f32Position[Z]));
  sOrient.s16Yaw = (Lint16)(f32NUMERICAL_Atan(f32DTan) * 10000);
  // value expressed accordingly the rloop system variable
  // http://confluence.rloop.org/display/SD/System+Variables
}


void vRecalcLateral(void) 
{
  Lfloat32 f32XDif = (Lfloat32)(sBeamLaser1.f32Position[Z] - sBeamLaser2.f32Position[Z]);
  Lfloat32 f32Coef =
      ((Lfloat32)(sBeamLaser2.f32Position[Z]) / f32XDif * sBeamLaser1.f32Measurement) -
      ((Lfloat32)(sBeamLaser1.f32Position[Z]) / f32XDif * sBeamLaser2.f32Measurement);
  sOrient.f32Lateral = f32Coef* f32NUMERICAL_Cosine((Lfloat32)(s16Yaw) / 10000.0);
}


void vCalculateTwist();
{
	
}
