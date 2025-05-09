/*
 * F-35 COMMON NOZZLE BOARD
 * Copyright (C) 2020-2022 Adam Williams <broadcast at earthling dot net>
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

// compute tables for N20 encoder positions
// make tables2;./tables2


#include <stdio.h>
#include <math.h>

#include "nozzle.h"





#define PATH "table.h"

// Inclusive range of encoder values.  
// MIN values are the HOME values when the nozzle is straight.
// MAX are encoder values when the nozzle is a right angle.
// We subtract the HOME values & interpolate from MIN to MAX to make
// tables.

#define ENCODER0_MAX 14097
#define ENCODER1_MAX 17681
#define ENCODER2_MAX 17801

int encoder0_values[PITCH_STEPS + 1];
int encoder1_values[PITCH_STEPS + 1];
int encoder2_values[PITCH_STEPS + 1];


void main()
{
    int i;
    
    
// need a sigmoid function instead of linear interpolation 
// since steps near the limits create bigger movements
    for(i = 0; i < PITCH_STEPS + 1; i++)
    {
        float sigma_min = -M_PI / 2;
        float sigma_max = M_PI;
        float sigma = sinh((double)i / PITCH_STEPS * (sigma_max - sigma_min) + sigma_min);
        float sigma0 = sinh(sigma_min);
        float sigma1 = sinh(sigma_max);
        sigma = (sigma - sigma0) / (sigma1 - sigma0);
printf("%f\n", sigma);
//        float sigma = (float)i / PITCH_STEPS;
        encoder0_values[i] = (ENCODER0_MAX - HOME0) * sigma;
        encoder1_values[i] = HOME1 + (ENCODER1_MAX - HOME1) * sigma;
        encoder2_values[i] = HOME2 + (ENCODER2_MAX - HOME2) * sigma;
    }


    printf("Writing %s\n", PATH);
    FILE *fd = fopen(PATH, "w");

    fprintf(fd, "// generated by tables2.c\n");

    fprintf(fd, "const int16_t pitch_to_encoders[] = \n{\n");
    for(i = 0; i < PITCH_STEPS + 1; i++)
    {
        fprintf(fd,
            "\t%d, %d, %d, \n",
            encoder0_values[i],
            encoder1_values[i],
            encoder2_values[i]);
    }
    fprintf(fd, "};\n\n");

    fclose(fd);
}












