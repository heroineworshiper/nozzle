/*
 * F-35 NOZZLE CONTROLLER
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

// compute tables for home made encoder positions
// gcc -o tables tables.c -lm
// ./tables


#include <stdio.h>
#include <math.h>

#define PATH "nozzle.X/nozzle.h"
#define TO_RAD(x) ((x) * 2 * M_PI / 360.0)
// total pitch steps inclusive
#define PITCH_STEPS 24

// inclusive range of encoder values
#define ENCODER0_MIN 6
#define ENCODER0_MAX 45

#define ENCODER1_MIN 17
#define ENCODER1_MAX 41

#define ENCODER2_MIN 8
#define ENCODER2_MAX 31

// home positions.  Not the same as the min & max
#define HOME0 22
#define HOME1 17
#define HOME2 8

// angle range of encoder 0 in the nozzle's current position
// move the nozzle or change the encoder limits & this has to be rotated
#define MIN_ANGLE TO_RAD(-190.0)
#define MAX_ANGLE TO_RAD(54.0)
// number of angle steps inclusive
#define ANGLE_STEPS (ENCODER0_MAX - ENCODER0_MIN)
// fixed point denominator
#define FIXED_D 128

// offsets that are added to encoder 0 for each pitch change
const int encoder0_values[] = 
{
    -10, 
    -10, 
    -10, 
    -10, 
    -10, 
    -10, 
    -10, 
    -9, 
    -9, 
    -8, 
    -8, 
    -7, 
    -6, 
    -6, 
    -5, 
    -5, 
    -4, 
    -4, 
    -3, 
    -3, 
    -2, 
    -2, 
    -1, 
    -1, 
    0, 
    0
};

int encoder1_values[PITCH_STEPS + 1];
int encoder2_values[PITCH_STEPS + 1];
int cos_table[ANGLE_STEPS + 1];
int sin_table[ANGLE_STEPS + 1];


void main()
{
    int i;
    
    
    
    for(i = 0; i < PITCH_STEPS + 1; i++)
    {
        encoder1_values[i] = ENCODER1_MIN + 
            (ENCODER1_MAX - ENCODER1_MIN) * i / PITCH_STEPS;
        encoder2_values[i] = ENCODER2_MIN + 
            (ENCODER2_MAX - ENCODER2_MIN) * i / PITCH_STEPS;
    }
    
    
    
// compute sin & cos for each encoder0 value
    for(i = 0; i < ANGLE_STEPS + 1; i++)
    {
        double angle = MIN_ANGLE + (MAX_ANGLE - MIN_ANGLE) * i / ANGLE_STEPS;
        double sin_value = sin(angle);
        double cos_value = cos(angle);
//        printf("angle=%f sin=%f cos=%f\n", angle, sin_value, cos_value);

        cos_table[i] = (int)(cos_value * FIXED_D);
        sin_table[i] = (int)(sin_value * FIXED_D);
    }

    
    printf("Writing %s\n", PATH);
    FILE *fd = fopen(PATH, "w");

    fprintf(fd, 
        "// generated by tables.c\n"
        "#define PITCH_STEPS %d\n"
        "#define ANGLE_STEPS %d\n"
        "\n"
        "#define ENCODER0_MIN %d\n"
        "#define ENCODER0_MAX %d\n"
        "\n"
        "#define ENCODER1_MIN %d\n"
        "#define ENCODER1_MAX %d\n"
        "\n"
        "#define ENCODER2_MIN %d\n"
        "#define ENCODER2_MAX %d\n"
        "\n"
        "#define HOME0 %d\n"
        "#define HOME1 %d\n"
        "#define HOME2 %d\n"
        "\n",
        PITCH_STEPS,
        ANGLE_STEPS,
        ENCODER0_MIN,
        ENCODER0_MAX,
        ENCODER1_MIN,
        ENCODER1_MAX,
        ENCODER2_MIN,
        ENCODER2_MAX,
        HOME0,
        HOME1,
        HOME2);

    fprintf(fd, "const int8_t step_to_encoders[] = \n{\n");
    for(i = 0; i < PITCH_STEPS + 1; i++)
    {
        fprintf(fd,
            "\t%d, %d, %d, \n",
            encoder0_values[i],
            encoder1_values[i],
            encoder2_values[i]);
    }
    fprintf(fd, "};\n\n");

    fprintf(fd, "const int8_t cos_table[] = \n{\n\t");
    for(i = 0; i < ANGLE_STEPS + 1; i++)
    {
        if(i > 0 && (i % 8) == 0)
        {
            fprintf(fd, "\n\t");
        }

        fprintf(fd,
            "%d, ",
            cos_table[i]);
    }
    fprintf(fd, "\n};\n\n");

    fprintf(fd, "const int8_t sin_table[] = \n{\n\t");
    for(i = 0; i < ANGLE_STEPS + 1; i++)
    {
        if(i > 0 && (i % 8) == 0)
        {
            fprintf(fd, "\n\t");
        }

        fprintf(fd,
            "%d, ",
            sin_table[i]);
    }
    fprintf(fd, "\n};\n\n");

    fclose(fd);
    
    
}









