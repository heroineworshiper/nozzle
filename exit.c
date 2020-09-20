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


// synthesize a nozzle exit
// gcc -g -o exit exit.c -lm;./exit




#include "3dstuff.h"


#define FILENAME "exit.stl"
#define SEGMENTS 15

#define REAR_RADIUS 50.0
#define FRONT_RADIUS 55.0

// angle between chevrons in degrees
#define GAP_ANGLE 0.0
// radius of gap
#define GAP_RADIUS1 45.0
#define GAP_RADIUS2 39.0


// same as tube radius
#define ATTACH_RADIUS 50.0
#define LENGTH 60.0

// length of chevrons
#define REAR_INSET 10
#define FRONT_INSET 5

// same as tube thickness
#define THICKNESS 2.0

void thickTriangle(vector coord1, vector coord2, vector coord3, int rearFace)
{
    vector thickness = { 0, THICKNESS, 0 };
// outer surface
    writeTriangle2(polarToXYZ(coord1), polarToXYZ(coord2), polarToXYZ(coord3));

// inner surface
    vector coord1_inner = subVectors(coord1, thickness);
    vector coord2_inner = subVectors(coord2, thickness);
    vector coord3_inner = subVectors(coord3, thickness);
    writeTriangle(polarToXYZ(coord1_inner), 
        polarToXYZ(coord2_inner), 
        polarToXYZ(coord3_inner));

// radial surface
    if(rearFace)
    {
        writeTriangle2(polarToXYZ(coord2_inner), 
            polarToXYZ(coord3_inner), 
            polarToXYZ(coord2));
        writeTriangle2(polarToXYZ(coord3_inner), 
            polarToXYZ(coord3), 
            polarToXYZ(coord2));
    }
    else
    {
        writeTriangle2(polarToXYZ(coord1_inner), 
            polarToXYZ(coord2_inner), 
            polarToXYZ(coord2));
        writeTriangle2(polarToXYZ(coord1), 
            polarToXYZ(coord1_inner), 
            polarToXYZ(coord2));
    }
}






void main()
{
    length = LENGTH;
    planeIntercept = length;


    double radius0 = FRONT_RADIUS;
    double radius1 = FRONT_RADIUS + 
        (REAR_RADIUS - FRONT_RADIUS) * 
        (FRONT_INSET / LENGTH);
    double radius2 = FRONT_RADIUS + 
        (GAP_RADIUS1 - FRONT_RADIUS) * 
        ((LENGTH - REAR_INSET) / LENGTH);
    double radius3 = FRONT_RADIUS + 
        (GAP_RADIUS2 - FRONT_RADIUS) * 
        ((LENGTH - REAR_INSET) / LENGTH);
    double radius4 = REAR_RADIUS;
    
    open_stl(FILENAME);


    int i, j;
    double segment_angle = toRad(360.0 / SEGMENTS);
    for(i = 0; i < SEGMENTS; i++)
//    for(i = 0; i < 1; i++)
    {
        double rear_z[] = {
            LENGTH - REAR_INSET,
            LENGTH,
            LENGTH - REAR_INSET 
        };
        
        double rear_a[] = {
            (double)i * segment_angle,
            (double)i * segment_angle + toRad(GAP_ANGLE) / 2,
            (double)i * segment_angle + segment_angle / 2,
            (double)(i + 1) * segment_angle - toRad(GAP_ANGLE) / 2,
            (double)(i + 1) * segment_angle
        };

        double front_z[] = {
            FRONT_INSET,
            0,
            FRONT_INSET,
            0,
            FRONT_INSET,
            0,
            FRONT_INSET,
            0,
            FRONT_INSET,
        };
        
        double front_a[] = 
        {
            (double)i * segment_angle,
            (double)i * segment_angle + segment_angle * 1 / 8,
            (double)i * segment_angle + segment_angle * 2 / 8,
            (double)i * segment_angle + segment_angle * 3 / 8,
            (double)i * segment_angle + segment_angle * 4 / 8,
            (double)i * segment_angle + segment_angle * 5 / 8,
            (double)i * segment_angle + segment_angle * 6 / 8,
            (double)i * segment_angle + segment_angle * 7 / 8,
            (double)i * segment_angle + segment_angle * 8 / 8
        };


        vector front_points[] = {
            { front_a[0], radius1, front_z[0] },
            { front_a[1], radius0, front_z[1] },
            { front_a[2], radius1, front_z[2] },
            { front_a[3], radius0, front_z[3] },
            { front_a[4], radius1, front_z[4] },
            { front_a[5], radius0, front_z[5] },
            { front_a[6], radius1, front_z[6] },
            { front_a[7], radius0, front_z[7] },
            { front_a[8], radius1, front_z[8] },
        };

        vector rear_points[] = {
            { rear_a[0], radius3, rear_z[0] },
            { rear_a[1], radius2, rear_z[0] },
            { rear_a[2], radius4, rear_z[1] },
            { rear_a[3], radius2, rear_z[2] },
            { rear_a[4], radius3, rear_z[2] },
        };

        for(j = 0; j < 2; j++)
        {
// gap
//            thickTriangle(front_points[0], rear_points[1], rear_points[0], 1);

            thickTriangle(front_points[0], rear_points[2], rear_points[1], 1);
            thickTriangle(front_points[0], front_points[1],  rear_points[2], 0);
            thickTriangle(front_points[1], front_points[2], rear_points[2], 0);
            thickTriangle(front_points[2], front_points[3], rear_points[2], 0);
//            thickTriangle(front_points[3], front_points[4], rear_points[2], 0);
//            thickTriangle(front_points[4], front_points[5], rear_points[2], 0);

// mirror
            thickTriangle(rear_points[2], front_points[3], front_points[4], 1);
            thickTriangle(rear_points[2], front_points[4], front_points[5], 1);
            thickTriangle(rear_points[2], front_points[5], front_points[6], 1);
            thickTriangle(rear_points[2], front_points[6], front_points[7], 1);
            thickTriangle(rear_points[2], front_points[7], front_points[8], 1);
            thickTriangle(rear_points[3], rear_points[2], front_points[8], 0);

// gap
//            thickTriangle(rear_points[4], rear_points[3], front_points[8], 0);
        }
    }


    close_stl();
}



