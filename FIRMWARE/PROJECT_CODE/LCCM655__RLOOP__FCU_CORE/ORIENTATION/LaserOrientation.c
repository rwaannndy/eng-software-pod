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
//	the frame of the pod has twisted.
// - will want to calc roll/pitch as seen by two sets of 3 lasers (a,b,c; b,c,d)
//	and return angle of twisting based on any discrepancy between the two measurements
// - check flags from optoncdt.c to verify that all lasers are functional.
//		- if one has failed, fall back to just pitch/roll with the 3 remaining


#include "LaserOrientation.h"

#define PI 3.14159265359

Lint16 s16Roll, s16Pitch, s16Yaw;
Lfloat32 f32Lateral;


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

// coordinate system: +x in direction of travel; +z up wrt gravity
void vLaserOrientation__Init(void)
{
	// TODO: All positions of components need their positions measured and assigned here.
		// blocked by installation of the components

	//Ground Facing Laser Positions
	sGroundLaser1.f32Position[3] = {8, 185, 35};
	sGroundLaser2.f32Position[3] = {-112, 18, 35};
	sGroundLaser3.f32Position[3] = {121, -53, 35};
	sGroundLaser4.f32Position[3] = {};

	// I-Beam laser positions
	sBeamLaser1.f32Position[1] = 25;  //x coord. laser 1
	sBeamLaser1.f32Position[2] = 0;   //y coord. laser 1
	sBeamLaser1.f32Position[3] = 35;  //z coord. laser 1
	sBeamLaser2.f32Position[1] = 25;  //x coord. laser 2
	sBeamLaser2.f32Position[2] = 100; //y coord. laser 2
	sBeamLaser2.f32Position[3] = 35;  //z coord. laser 2

	//Hover Engine Positions {x,y,z} (from top view)
	sHE1.f32Position[3] = {61, 130, 0}; // Top Left
	sHE2.f32Position[3] = {62, 129, 0}; // Top Right
	sHE3.f32Position[3] = {62, 126, 0}; // Bottom Right
	sHE4.f32Position[3] = {60, 128, 0}; // Bottom Left

	// TODO: These engines are not yet implemented in this code.
	sHE5.f32Position[3] = {0, 0, 0}; // Top Left
	sHE6.f32Position[3] = {0, 0, 0}; // Top Right
	sHE7.f32Position[3] = {0, 0, 0}; // Bottom Right
	sHE8.f32Position[3] = {0, 0, 0}; // Bottom Left

	//vPrintPlane();

}

//Recalculate the orientation and engine heights
void vLaserOrientation__Process(void)
{
    vCalculateGroundPlane(sGroundLaser1, sGroundLaser2, sGroundLaser3);

	sHE1.f32Measurement = f32PointToPlaneDistance(sHE1.f32Position);
	sHE2.f32Measurement = f32PointToPlaneDistance(sHE2.f32Position);
	sHE3.f32Measurement = f32PointToPlaneDistance(sHE3.f32Position);
	sHE4.f32Measurement = f32PointToPlaneDistance(sHE4.f32Position);
	vRecalcPitch();
	vRecalcRoll();
}

//Basically the vehicle is a static reference
//and we recalculate the orientation of the
//ground plane relative to the vehicle
//and the hover engines
Lfloat32 f32PlaneCoeffs[4]; //TODO: Check this size   // ordered as: A, B, C, D, decreasing polynomial terms

void vRecalcRoll();
void vRecalcPitch();

Lfloat32 f32PointToPlaneDistance(Lfloat32 f32Position[3])
{
	return (f32PlaneCoeffs[0] * f32Position[0] + f32PlaneCoeffs[1] * f32Position[1] + f32PlaneCoeffs[2] * f32Position[2] + f32PlaneCoeffs[3]) / sqrt((double)(f32PlaneCoeffs[0] * f32PlaneCoeffs[0] + f32PlaneCoeffs[1] * f32PlaneCoeffs[1] + f32PlaneCoeffs[2] * f32PlaneCoeffs[2]));
}


//The angle between two planes that yields the roll
void vRecalcRoll(void)
{
	//Normal vector of the other plane
	Lfloat32 f32vec1x = 1, f32vec1y = 0, f32vec1z = 0;

	//Angle between two planes // TODO: Need to find a Lachlan func for this
	s16Roll = (Lint16)(acos((double)((f32vec1x * f32PlaneCoeffs[0] + f32vec1y * f32PlaneCoeffs[1] + f32vec1z * f32PlaneCoeffs[2]) / sqrt((double)(f32PlaneCoeffs[0] * f32PlaneCoeffs[0] + f32PlaneCoeffs[1] * f32PlaneCoeffs[1] + f32PlaneCoeffs[2] * f32PlaneCoeffs[2])))) * 10000);
}

//The angle between two planes that yields the pitch
void vRecalcPitch(void)
{
	//Normal vector of the other plane
	Lfloat32 f32vec1x = 0, f32vec1y = 1, f32vec1z = 0;

	//Angle between two planes // TODO: Need to find a Lachlan func for this
	s16Pitch = (Lint16)(acos((double)((f32vec1x * f32PlaneCoeffs[0] + f32vec1y * f32PlaneCoeffs[1] + f32vec1z * f32PlaneCoeffs[2]) / sqrt((double)(f32PlaneCoeffs[0] * f32PlaneCoeffs[0] + f32PlaneCoeffs[1] * f32PlaneCoeffs[1] + f32PlaneCoeffs[2] * f32PlaneCoeffs[2])))) * 10000);
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
	f32Vec1X = sGroundLaser1.f32Position[0] - sGroundLaser2.f32Position[0];
	f32Vec1Y = sGroundLaser1.f32Position[1] - sGroundLaser2.f32Position[1];
	f32Vec1Z = (sGroundLaser1.f32Position[2] - sGroundLaser1.f32Measurement) - (sGroundLaser2.f32Position[2] - sGroundLaser2.f32Measurement);
	f32Vec2X = sGroundLaser2.f32Position[0] - sGroundLaser3.f32Position[0];
	f32Vec2Y = sGroundLaser2.f32Position[1] - sGroundLaser3.f32Position[1];
	f32Vec2Z = (sGroundLaser2.f32Position[2] - sGroundLaser2.f32Measurement) - (sGroundLaser3.f32Position[2] - sGroundLaser3.f32Measurement);

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
	f32PlaneCoeffs[3] = -1 * (f32XProductX * sGroundLaser1.f32Position[0] + f32XProductY * sGroundLaser1.f32Position[1] + f32XProductZ * sGroundLaser1.f32Position[2]);

	f32PlaneCoeffs[0] = f32XProductX;
	f32PlaneCoeffs[1] = f32XProductY;
	f32PlaneCoeffs[2] = f32XProductZ;
}


void vRecalcYaw(void) 
{
  Lfloat32 f32SDif = (Lfloat32)(sBeamLaser1.f32Measurement - sBeamLaser2.f32Measurement);
  Lfloat32 f32DTan = f32SDif / ((Lfloat32)(sBeamLaser1.f32Position[2] - sBeamLaser2.f32Position[2]));
  s16Yaw = (Lint16)(f32NUMERICAL_Atan(f32DTan) * 10000);
  // value expressed accordingly the rloop system variable
  // http://confluence.rloop.org/display/SD/System+Variables
}


void vRecalcLateral(void) 
{
  Lfloat32 f32XDif = (Lfloat32)(sBeamLaser1.f32Position[2] - sBeamLaser2.f32Position[2]);
  Lfloat32 f32Coef=
      ((Lfloat32)(sBeamLaser2.f32Position[2]) / f32XDif * sBeamLaser1.f32Measurement) -
      ((Lfloat32)(sBeamLaser1.f32Position[2]) / f32XDif * sBeamLaser2.f32Measurement);
  f32Lateral = f32Coef* f32NUMERICAL_Cosine((Lfloat32)(s16Yaw) / 10000.0);
}
