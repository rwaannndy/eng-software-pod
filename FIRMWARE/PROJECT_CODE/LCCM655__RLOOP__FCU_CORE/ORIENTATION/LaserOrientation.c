/* TODO:
 * doxygen compatible commenting
 */

 /**
 * @file		LaserOrientation.c
 * @brief		Orientation and Position from Distance Lasers
 * @author		David
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

Lfloat32 Roll, Pitch;


//All units in mm
//The math doesn't care as long as you're consistent

//For the laser positions Z should be the reading when
//the HDK is sitting flat on the 4 hover engines

// set laser structs
struct _strComponent sLaser1;
struct _strComponent sLaser2;
struct _strComponent sLaser3;

// set hover engine structs
struct _strComponent sHE1;
struct _strComponent sHE2;
struct _strComponent sHE3;
struct _strComponent sHE4;
struct _strComponent sHE5;
struct _strComponent sHE6;
struct _strComponent sHE7;
struct _strComponent sHE8;


void vLaserOrientation__Init(void)
{
	// TODO: All positions of components need their positions measured and assigned here.
		// blocked by installation of the components

	//Laser Positions
	sLaser1.f32Position[3] = {8, 185, 35};
	sLaser2.f32Position[3] = {-112, 18, 35};
	sLaser3.f32Position[3] = {121, -53, 35};

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

	vPrintPlane();

}

void vLaserOrientation__Process(void)
{
		CalculateGroundPlane(struct sLaser1, struct sLaser2, struct sLaser3);
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

//After the laser readings are updated this function
//will recalculate the orientation and engine heights
void vRecalcOrientation(void)
{
    CalculateGroundPlane(Laser1X, Laser1Y, Laser1Z - Laser1Reading, Laser2X, Laser2Y, Laser2Z - Laser2Reading, Laser3X, Laser3Y, Laser3Z - Laser3Reading);

	sHE1.f32Measurement = PointToPlaneDistance(sHE1.f32Position);
	sHE2.f32Measurement = PointToPlaneDistance(sHE2.f32Position);
	sHE3.f32Measurement = PointToPlaneDistance(sHE3.f32Position);
	sHE4.f32Measurement = PointToPlaneDistance(sHE4.f32Position);
	RecalcPitch();
	RecalcRoll();
}

//The angle between two planes that yields the roll
void vRecalcRoll(void)
{
	//Normal vector of the other plane
	Lfloat32 f32vec1x = 1, f32vec1y = 0, f32vec1z = 0;

	//Angle between two planes // TODO: Need to find a Lachlan func for this 
	Roll = acos((double)((f32vec1x * f32PlaneCoeffs[0] + f32vec1y * f32PlaneCoeffs[1] + f32vec1z * f32PlaneCoeffs[2]) / sqrt((double)(f32PlaneCoeffs[0] * f32PlaneCoeffs[0] + f32PlaneCoeffs[1] * f32PlaneCoeffs[1] + f32PlaneCoeffs[2] * f32PlaneCoeffs[2])))) * 180/PI;
}

//The angle between two planes that yields the pitch
void vRecalcPitch(void)
{
	//Normal vector of the other plane
	Lfloat32 f32vec1x = 0, f32vec1y = 1, f32vec1z = 0;

	//Angle between two planes // TODO: Need to find a Lachlan func for this 
	Pitch = acos((double)((f32vec1x * f32PlaneCoeffs[0] + f32vec1y * f32PlaneCoeffs[1] + f32vec1z * f32PlaneCoeffs[2]) / sqrt((double)(f32PlaneCoeffs[0] * f32PlaneCoeffs[0] + f32PlaneCoeffs[1] * f32PlaneCoeffs[1] + f32PlaneCoeffs[2] * f32PlaneCoeffs[2])))) * 180 / PI;
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
	Lfloat32 f32d;

	//Calculate two vectors in the plane
	f32Vec1X = sLaser1.f32Position[0] - sLaser2.f32Position[0];
	f32Vec1Y = sLaser1.f32Position[1] - sLaser2.f32Position[1];
	f32Vec1Z = (sLaser1.f32Position[2] - sLaser1.f32Measurement) - (sLaser2.f32Position[2] - sLaser2.f32Measurement);
	f32Vec2X = sLaser2.f32Position[0] - sLaser3.f32Position[0];
	f32Vec2Y = sLaser2.f32Position[1] - sLaser3.f32Position[1];
	f32Vec2Z = (sLaser2.f32Position[2] - sLaser2.f32Measurement) - (sLaser3.f32Position[2] - sLaser3.f32Measurement);

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
	d = -1 * (f32XProductX * sLaser1.f32Position[0] + f32XProductY * sLaser1.f32Position[1] + f32XProductZ * sLaser1.f32Position[2]);

	f32PlaneCoeffs[0] = f32XProductX;
	f32PlaneCoeffs[1] = f32XProductY;
	f32PlaneCoeffs[2] = f32XProductZ;
	f32PlaneCoeffs[3] = f32d;
}
