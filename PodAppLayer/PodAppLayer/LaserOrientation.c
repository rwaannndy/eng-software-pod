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

//If we are using anything involving floating point trig and that trig is safety critical (i.e. pod distance or braking) 
 //then we must also do a parallel equation in another data type to prevent the sorts of errors that are commonly seen with floating point trig.


#include "LaserOrientation.h"

#define PI 3.14159265359

float Roll, Pitch;

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
	sLaser1.f16Position[3] = {8, 185, 35};
	sLaser2.f16Position[3] = {-112, 18, 35};
	sLaser3.f16Position[3] = {121, -53, 35};

	//Hover Engine Positions {x,y,z} (from top view)
	sHE1.f16Position[3] = {61, 130, 0}; // Top Left
	sHE2.f16Position[3] = {62, 129, 0}; // Top Right
	sHE3.f16Position[3] = {62, 126, 0}; // Bottom Right
	sHE4.f16Position[3] = {60, 128, 0}; // Bottom Left

	// TODO: Needs to be implemented:
	sHE5.f16Position[3] = {0, 0, 0}; // Top Left
	sHE6.f16Position[3] = {0, 0, 0}; // Top Right
	sHE7.f16Position[3] = {0, 0, 0}; // Bottom Right
	sHE8.f16Position[3] = {0, 0, 0}; // Bottom Left

	PrintPlane();

}

void vLaserOrientation__Process(void)
{
		CalculateGroundPlane(struct sLaser1, struct sLaser2, struct sLaser3);
}

//Basically the vehicle is a static reference
//and we recalculate the orientation of the
//ground plane relative to the vehicle
//and the hover engines
Lfloat16 f16PlaneCoeffs[4]; //TODO: Check this size   // ordered as: A, B, C, D, decreasing polynomial terms

void vRecalcRoll();
void vRecalcPitch();

Lfloat16 f16PointToPlaneDistance(Lfloat16 f16Position[3])
{
	return (f16PlaneCoeffs[0] * f16Position[0] + f16PlaneCoeffs[1] * f16Position[1] + f16PlaneCoeffs[2] * f16Position[2] + f16PlaneCoeffs[3]) / sqrt((double)(f16PlaneCoeffs[0] * f16PlaneCoeffs[0] + f16PlaneCoeffs[1] * f16PlaneCoeffs[1] + f16PlaneCoeffs[2] * f16PlaneCoeffs[2]));
}

//After the laser readings are updated this function
//will recalculate the orientation and engine heights
void vRecalcOrientation(void)
{
    CalculateGroundPlane(Laser1X, Laser1Y, Laser1Z - Laser1Reading, Laser2X, Laser2Y, Laser2Z - Laser2Reading, Laser3X, Laser3Y, Laser3Z - Laser3Reading);

	sHE1.f16Measurement = PointToPlaneDistance(sHE1.f16Position);
	sHE2.f16Measurement = PointToPlaneDistance(sHE2.f16Position);
	sHE3.f16Measurement = PointToPlaneDistance(sHE3.f16Position);
	sHE4.f16Measurement = PointToPlaneDistance(sHE4.f16Position);
	RecalcPitch();
	RecalcRoll();
}

//The angle between two planes that yields the roll
void vRecalcRoll(void)
{
	//Normal vector of the other plane
	float vec1x = 1, vec1y = 0, vec1z = 0;

	//Angle between two planes // TODO: Need to find a Lachlan file for this 
	Roll = acos((double)((vec1x * f16PlaneCoeffs[0] + vec1y * f16PlaneCoeffs[1] + vec1z * f16PlaneCoeffs[2]) / sqrt((double)(f16PlaneCoeffs[0] * f16PlaneCoeffs[0] + f16PlaneCoeffs[1] * f16PlaneCoeffs[1] + f16PlaneCoeffs[2] * f16PlaneCoeffs[2])))) * 180/PI;
}

//The angle between two planes that yields the pitch
void vRecalcPitch(void)
{
	//Normal vector of the other plane
	float vec1x = 0, vec1y = 1, vec1z = 0;

	//Angle between two planes // TODO: Need to find a Lachlan file for this 
	Pitch = acos((double)((vec1x * f16PlaneCoeffs[0] + vec1y * f16PlaneCoeffs[1] + vec1z * f16PlaneCoeffs[2]) / sqrt((double)(f16PlaneCoeffs[0] * f16PlaneCoeffs[0] + f16PlaneCoeffs[1] * f16PlaneCoeffs[1] + f16PlaneCoeffs[2] * f16PlaneCoeffs[2])))) * 180 / PI;
}

void vPrintPlane(void)
{
	// printf("A:%f B:%f C:%f D:%f\n", f16PlaneCoeffs[0], f16PlaneCoeffs[1], f16PlaneCoeffs[2], f16PlaneCoeffs[3]);
}

//Calculate the ground plane given three points
//Ax + By + Cz + D = 0
void vCalculateGroundPlane(struct sLaserA, struct sLaserB, struct sLaserC)
{
	float Vec1X, Vec1Y, Vec1Z;
	float Vec2X, Vec2Y, Vec2Z;
	float XProductX, XProductY, XProductZ;
	float d;

	//Calculate two vectors in the plane
	Vec1X = sLaser1.f16Position[0] - sLaser2.f16Position[0];
	Vec1Y = sLaser1.f16Position[1] - sLaser2.f16Position[1];
	Vec1Z = (sLaser1.f16Position[2] - sLaser1.f16Measurement) - (sLaser2.f16Position[2] - sLaser2.f16Measurement);
	Vec2X = sLaser2.f16Position[0] - sLaser3.f16Position[0];
	Vec2Y = sLaser2.f16Position[1] - sLaser3.f16Position[1];
	Vec2Z = (sLaser2.f16Position[2] - sLaser2.f16Measurement) - (sLaser3.f16Position[2] - sLaser3.f16Measurement);

	//Calculate the cross product of the vectors
	//to get a vector normal to the plane
	XProductX = Vec1Y*Vec2Z - Vec1Z*Vec2Y;
	XProductY = Vec1Z*Vec2X - Vec1X*Vec2Z;
	XProductZ = Vec1X*Vec2Y - Vec1Y*Vec2X;

	//The normal vector should be pointed in the +Z direction
	//It affects which side of the plane has negative distances
	if (XProductZ < 0){
		XProductX *= -1;
		XProductY *= -1; 
		XProductZ *= -1;
	}

	//Plane in 3D: Ax + By + Cz + D = 0
	//A, B, C is the vector normal to the plane
	//Use one of our original points to calculate D
	d = -1 * (XProductX * sLaser1.f16Position[0] + XProductY * sLaser1.f16Position[1] + XProductZ * sLaser1.f16Position[2]);

	f16PlaneCoeffs[0] = XProductX;
	f16PlaneCoeffs[1] = XProductY;
	f16PlaneCoeffs[2] = XProductZ;
	f16PlaneCoeffs[3] = d;
}
