/*
 * F-35 NOZZLE CONTROLLER
 * Copyright (C) 2020 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */


// synthesize an isogrid tube & write to an STL file
// gcc -g -O2 -o tube tube.c -lm
// ./tube inlet.stl 130 0
// ./tube elbow.stl 60 -22.5
// ./tube outlet.stl 80 -22.5



#include "3dstuff.h"





// mm
// cylinder dimensions
#define RADIUS 50.0
#define THICKNESS 1.0

#define TOP_SKIRT_LENGTH 3.0
#define BOTTOM_SKIRT_LENGTH 2.0
#define SKIRT_INSET 0.0
#define SKIRT_OUTSET 2.0

// isogrid dimensions
#define SLICE_LENGTH 10.0

// multiple of 2 per slice
#define ISOGRID_SEGMENTS 20
#define RIB_WIDTH 2.0
#define RIB_DEPTH 2.0
#define RIB_DIVISIONS 5
//#define RIB_DIVISIONS 1

#define TUBE_SEGMENTS (360 / 5)
// print only a slice of the tube by changing these
//#define USE_CLIPPING
    #define MIN_ANGLE 103
    #define MAX_ANGLE 143
    #define MIN_Z 0
    #define MAX_Z 30



// calculated values
vector planeNormal;
vector planePoint;
int slices;

// clip line in polar coordinates to the clipping plane
// returns 1 if the entire line is above the plane
int clipLine(vector *point1, vector *point2)
{
    vector xyz1 = polarToXYZ(*point1);
    vector xyz2 = polarToXYZ(*point2);

    double planeZ1 = planeIntercept + xyz1.x * planeSlope;
    double planeZ2 = planeIntercept + xyz2.x * planeSlope;

#ifdef USE_CLIPPING
    planeZ1 = MAX_Z;
    planeZ2 = MAX_Z;

    if(point1->x < toRad(MIN_ANGLE) || point1->x >= toRad(MAX_ANGLE) ||
        point2->x < toRad(MIN_ANGLE) || point2->x >= toRad(MAX_ANGLE))
    {
        return 1;
    }

#endif // USE_CLIPPING

   if(xyz1.z > planeZ1 && xyz2.z > planeZ2)
   {
// printf("clipLine %d xyz1.z=%f planeZ1=%f xyz2.z=%f planeZ2=%f\n",
// __LINE__,
// xyz1.z,
// planeZ1,
// xyz2.z,
// planeZ2);
       return 1;
   }

    if(xyz1.z > planeZ1)
    {
        // intersection between line & plane
#ifdef USE_CLIPPING
//             vector intersection = intersectionPoint(
//                 subVectors(xyz2, xyz1), 
//                 xyz1, 
//                 (vector){ 0.0, 0.0, 1.0 }, 
//                 (vector){ 0.0, 0.0, MAX_Z });
        vector intersection = intersectionPoint2(
            xyz1,
            xyz2,
            MAX_Z,
            0);
#else
//             vector intersection = intersectionPoint(
//                 subVectors(xyz2, xyz1), 
//                 xyz1, 
//                 planeNormal, 
//                 planePoint);
        vector intersection = intersectionPoint2(
            xyz1,
            xyz2,
            planeIntercept,
            planeSlope);
#endif

        *point1 = XYZToPolar(intersection);
    }

    if(xyz2.z > planeZ2)
    {
        // intersection between line & plane
#ifdef USE_CLIPPING
//             vector intersection = intersectionPoint(
//                 subVectors(xyz2, xyz1), 
//                 xyz2, 
//                 (vector){ 0.0, 0.0, 1.0 }, 
//                 (vector){ 0.0, 0.0, MAX_Z });
        vector intersection = intersectionPoint2(
            xyz1,
            xyz2,
            MAX_Z,
            0);
#else
//             vector intersection = intersectionPoint(
//                 subVectors(xyz2, xyz1), 
//                 xyz1, 
//                 planeNormal, 
//                 planePoint);
        vector intersection = intersectionPoint2(
            xyz1,
            xyz2,
            planeIntercept,
            planeSlope);
#endif

        *point2 = XYZToPolar(intersection);
    }

    return 0;
}


void loftQuads(vector *topCoords, vector *bottomCoords)
{

// if(triangleCount < 84)
// {
//     printf("loftQuads %d: top:%f,%f,%f %f,%f,%f %f,%f,%f %f,%f,%f\n",
//         __LINE__,
//         topCoords[0].x,
//         topCoords[0].y,
//         topCoords[0].z,
//         topCoords[1].x,
//         topCoords[1].y,
//         topCoords[1].z,
//         topCoords[2].x,
//         topCoords[2].y,
//         topCoords[2].z,
//         topCoords[3].x,
//         topCoords[3].y,
//         topCoords[3].z);
//     printf("loftQuads %d: bottom:%f,%f,%f %f,%f,%f %f,%f,%f %f,%f,%f\n",
//         __LINE__,
//         bottomCoords[0].x,
//         bottomCoords[0].y,
//         bottomCoords[0].z,
//         bottomCoords[1].x,
//         bottomCoords[1].y,
//         bottomCoords[1].z,
//         bottomCoords[2].x,
//         bottomCoords[2].y,
//         bottomCoords[2].z,
//         bottomCoords[3].x,
//         bottomCoords[3].y,
//         bottomCoords[3].z);
// }

// top
    writeQuad(topCoords[3], topCoords[2], topCoords[1], topCoords[0]);

// sides
    writeQuad(topCoords[0], topCoords[1], bottomCoords[1], bottomCoords[0]);
    writeQuad(topCoords[1], topCoords[2], bottomCoords[2], bottomCoords[1]);
    writeQuad(topCoords[2], topCoords[3], bottomCoords[3], bottomCoords[2]);
    writeQuad(topCoords[3], topCoords[0], bottomCoords[0], bottomCoords[3]);

// bottom
    writeQuad(bottomCoords[0], bottomCoords[1], bottomCoords[2], bottomCoords[3]);
}

// create a rib from a path
void pathToRib(vector *path, int pathSize)
{
    vector *edgeLoops[pathSize];
    int i, j;
    int loopPoints = 4;
    int totalLoops = 0;
    vector depthVector = (vector){ 0, RIB_DEPTH, 0.0 };

    if(pathSize < 2)
    {
        return;
    }

// always create pathSize loops
    for(i = 0; i < pathSize - 1; i++)
    {
        vector point1 = path[i];
        vector point2 = path[i + 1];
        vector *edgeLoop = calloc(sizeof(vector), loopPoints);
        
        // get perpendicular vector on the surface of the cylinder
        vector diff = subVectors(point1, point2);
        double perpX = diff.z;
        double perpZ = sin(diff.x) * RADIUS;
        double perpMag = hypot(perpX, perpZ);
        perpX = perpX / perpMag * RIB_WIDTH / 2;
        perpZ = perpZ / perpMag * RIB_WIDTH / 2;
        vector perpVector = { -asin(perpX / RADIUS), 0, perpZ };
        
        edgeLoop[3] = addVectors(addVectors(point1, perpVector), depthVector);
        edgeLoop[2] = addVectors(point1, perpVector);
        edgeLoop[1] = subVectors(point1, perpVector);
        edgeLoop[0] = addVectors(subVectors(point1, perpVector), depthVector);
        edgeLoops[i] = edgeLoop;

// final loop
        if(i == pathSize - 2)
        {
            vector *edgeLoop = calloc(sizeof(vector), loopPoints);
            edgeLoop[3] = addVectors(addVectors(point2, perpVector), depthVector);
            edgeLoop[2] = addVectors(point2, perpVector);
            edgeLoop[1] = subVectors(point2, perpVector);
            edgeLoop[0] = addVectors(subVectors(point2, perpVector), depthVector);
            edgeLoops[i + 1] = edgeLoop;
        }
    }

// clip loops.  Shit show.
//     int done = 0;
//     for(i = 0; i < pathSize - 1 && !done; i++)
//     {
//         vector *edgeLoop1 = edgeLoops[i];
//         vector *edgeLoop2 = edgeLoops[i + 1];
// 
// 
//         for(j = 0; j < loopPoints; j++)
//         {
//             if(clipLine(&edgeLoop1[j], &edgeLoop2[j]))
//             {
//                 done = 1;
//                 break;
//             }
//         }
// 
//         if(!done)
//         {
//             totalLoops++;
//         }
//     }
//     totalLoops++;

    totalLoops = pathSize;

    if(totalLoops > 1)
    {
        for(i = 0; i < totalLoops; i++)
        {
            for(j = 0; j < loopPoints; j++)
            {
                edgeLoops[i][j] = polarToXYZ(edgeLoops[i][j]);
            }
        }
        loopsToSolid(edgeLoops, totalLoops, loopPoints, 1);
    }
    
    for(i = 0; i < pathSize; i++)
    {
        free(edgeLoops[i]);
    }
}





void makeIsogrid()
{
// tube is divided horizontally into segments
    int segment;
    int pass;
    
    for(pass = 0; pass < 2; pass++)
    {
        for(segment = 0; segment < ISOGRID_SEGMENTS; segment += 2)
        {
// tube is divided vertically into slices
            int slice;
            int slice1 = -1;
            int slice2 = -1;
            vector path[slices * RIB_DIVISIONS + 1];
            for(slice = 0; slice < slices * RIB_DIVISIONS + 1; slice++)
            {
                double angle;
                if(pass == 0)
                {
                    angle = toRad((double)((double)segment - (double)slice / RIB_DIVISIONS) * 
                        360.0 / 
                        ISOGRID_SEGMENTS);
                }
                else
                {
                    angle = toRad((double)((double)segment + (double)slice / RIB_DIVISIONS) * 
                        360.0 / 
                        ISOGRID_SEGMENTS);
                }
                
                path[slice] = (vector){ 
                    angle, 
                    RADIUS, 
                    slice * length / slices / RIB_DIVISIONS
                };

// clip path
                if(slice > 0 && 
                    !clipLine(&path[slice - 1], &path[slice]))
                {
// get the 1st point
                    if(slice1 < 0)
                    {
                        slice1 = slice - 1;
                    }
// current point is the last point
                    slice2 = slice + 1;
                }
            }


            if(slice2 > slice1)
            {
                pathToRib(&path[slice1], slice2 - slice1);
            }
        }
    }

// longerons
    for(segment = 0; segment < ISOGRID_SEGMENTS; segment++)
    {
        double angle = toRad((double)segment * 360.0 / ISOGRID_SEGMENTS);
        vector path[2];
        path[0] = (vector){ angle, RADIUS, 0 };
        path[1] = (vector){ angle, RADIUS, length };
        
        if(!clipLine(&path[0], &path[1]))
        {
            pathToRib(path, 2);
        }
    }

}



void makeTube()
{
    int i, j;
    double step = 360.0 / TUBE_SEGMENTS;

// edge loops defining the wall of the tube
    vector* edgeLoops[TUBE_SEGMENTS];
    int totalLoops = 0;
    int loopPoints = 4;
    
    for(i = 0; i < TUBE_SEGMENTS; i++)
    {
        double angle = toRad((double)i * 360.0 / TUBE_SEGMENTS);
        vector *edgeLoop = calloc(sizeof(vector), loopPoints);

        vector topInner = { angle, RADIUS, length };
        vector topOuter = { angle, RADIUS + THICKNESS, length };
        vector bottomInner = { angle, RADIUS };
        vector bottomOuter = { angle, RADIUS + THICKNESS };

        if(clipLine(&bottomInner, &topInner) || 
            clipLine(&bottomOuter, &topOuter))
        {
            free(edgeLoop);
            continue;
        }

        edgeLoop[0] = topOuter;
        edgeLoop[1] = topInner;
        edgeLoop[2] = bottomInner;
        edgeLoop[3] = bottomOuter;

        edgeLoops[totalLoops] = edgeLoop;
        totalLoops++;
    }
//printf("makeTube %d totalLoops=%d\n", __LINE__, totalLoops);

// create skirt
    vector topSkirtOffset = { 0, 0, TOP_SKIRT_LENGTH / 2 };
    vector bottomSkirtOffset = { 0, 0, BOTTOM_SKIRT_LENGTH / 2 };
    vector* topSkirt[totalLoops];
    vector* bottomSkirt[totalLoops];
    int topSkirtLoops = 0;
    int bottomSkirtLoops = 0;
    for(i = 0; i < TUBE_SEGMENTS; i++)
    {
        double angle = toRad((double)i * 360.0 / TUBE_SEGMENTS);
        vector *topLoop = calloc(sizeof(vector), loopPoints);
        vector *bottomLoop = calloc(sizeof(vector), loopPoints);
        vector *srcLoop = edgeLoops[i];
    
        vector topInner =    { angle, RADIUS - SKIRT_INSET,  length + RIB_DEPTH };
        vector topOuter =    { angle, RADIUS + SKIRT_OUTSET, length + RIB_DEPTH };
// bottom coords are needed to calculate the clipping
        vector bottomInner = { angle, RADIUS - SKIRT_INSET,  0 };
        vector bottomOuter = { angle, RADIUS + SKIRT_OUTSET, 0 };
        if(clipLine(&bottomOuter, &topOuter) ||
            clipLine(&bottomInner, &topInner))
        {
            free(topLoop);
        }
        else
        {
// add thickness to skirt in XYZ coordinates to make it ignore the aspect ratio
            topInner = polarToXYZ(topInner);
            topOuter = polarToXYZ(topOuter);

            topLoop[0] = addVectors(topOuter, topSkirtOffset);
            topLoop[1] = addVectors(topInner, topSkirtOffset);
            topLoop[2] = subVectors(topInner, topSkirtOffset);
            topLoop[3] = subVectors(topOuter, topSkirtOffset);

// can't 3D print a bottom edge with overhang
            if(topLoop[3].z < topLoop[2].z + 0.5)
            {
                topLoop[3].z = topLoop[2].z + 0.5;
            }

            topSkirt[topSkirtLoops] = topLoop;
            topSkirtLoops++;
        }
        
        
        bottomInner = (vector){ angle, RADIUS - SKIRT_INSET,  0};
        bottomOuter = (vector){ angle, RADIUS + SKIRT_OUTSET, 0 };
// top coords are needed to calculate the clipping
        topInner =    (vector){ angle, RADIUS - SKIRT_INSET,  length + RIB_DEPTH };
        topOuter =    (vector){ angle, RADIUS + SKIRT_OUTSET, length + RIB_DEPTH };
        
        if(clipLine(&bottomOuter, &topOuter) ||
            clipLine(&bottomInner, &topInner))
        {
            free(bottomLoop);
        }
        else
        {
            bottomInner = polarToXYZ(bottomInner);
            bottomOuter = polarToXYZ(bottomOuter);
            bottomLoop[0] = addVectors(bottomOuter, bottomSkirtOffset);
            bottomLoop[1] = addVectors(bottomInner, bottomSkirtOffset);
            bottomLoop[2] = subVectors(bottomInner, bottomSkirtOffset);
            bottomLoop[3] = subVectors(bottomOuter, bottomSkirtOffset);
            bottomSkirt[bottomSkirtLoops] = bottomLoop;
            bottomSkirtLoops++;
        }
        
    }

// polar to XYZ
    for(i = 0; i < totalLoops; i++)
    {
        for(j = 0; j < loopPoints; j++)
        {
            edgeLoops[i][j] = polarToXYZ(edgeLoops[i][j]);
        }
    }

// create solids from edgeloops
    loopsToSolid(edgeLoops, totalLoops, loopPoints, 0);
    loopsToSolid(topSkirt, topSkirtLoops, loopPoints, 0);
    loopsToSolid(bottomSkirt, bottomSkirtLoops, loopPoints, 0);

    for(i = 0; i < totalLoops; i++)
    {
        free(edgeLoops[i]);
    }

    for(i = 0; i < topSkirtLoops; i++)
    {
        free(topSkirt[i]);
    }

    for(i = 0; i < bottomSkirtLoops; i++)
    {
        free(bottomSkirt[i]);
    }
}


void main(int argc, char *argv[])
{
    if(argc < 4)
    {
        printf("Usage: tube <filename> <length> <plane angle>\n");
        exit(1);
    }
    
    
    char *path = argv[1];
    length = atof(argv[2]);
    planeAngle = toRad(atof(argv[3]));

    printf("path=%s length=%f planeAngle=%f\n",
        path,
        length,
        planeAngle);
    slices = ((int)(length / SLICE_LENGTH));


// slope of the clipping plane Z/X
    planeSlope = tan(planeAngle);
// where clipping plane intersects Z axis
    planeIntercept = length + planeSlope * (RADIUS + RIB_DEPTH);

    open_stl(path);

    planeNormal = (vector){ 
        -sin(planeAngle), 
        0, 
        cos(planeAngle) 
    };
    planePoint = (vector){ 0.0, 0.0, planeIntercept };



// slope vs X
//     double x = -RADIUS;
//     for(x = -RADIUS; x <= RADIUS; x += 10.0)
//     {
// //        double topX = x * cos(planeAngle);
// //        double topZ = planeIntercept + x * sin(planeAngle);
// //        double slope = (topX - x) / topZ;
// //        printf("main %d x=%f slope=%f\n", __LINE__, x, slope);
//         vector point = { M_PI / 4, x, length / 2 };
//         vector point2 = polarToXYZ(point);
//         vector point3 = XYZToPolar(point2);
//         printf("main %d point=%f,%f,%f point3=%f,%f,%f\n",
//             __LINE__,
//             point.x, point.y, point.z,
//             point3.x, point3.y, point3.z);
//     }


    makeIsogrid();
    makeTube();

    close_stl();
}



