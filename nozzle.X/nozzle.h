// generated by tables.c
#define PITCH_STEPS 24
#define ANGLE_STEPS 39

#define ENCODER0_MIN 6
#define ENCODER0_MAX 45

#define ENCODER1_MIN 17
#define ENCODER1_MAX 41

#define ENCODER2_MIN 8
#define ENCODER2_MAX 31

#define HOME0 22
#define HOME1 17
#define HOME2 8

const int8_t step_to_encoders[] = 
{
	-10, 17, 8, 
	-10, 18, 8, 
	-10, 19, 9, 
	-10, 20, 10, 
	-10, 21, 11, 
	-10, 22, 12, 
	-10, 23, 13, 
	-9, 24, 14, 
	-9, 25, 15, 
	-8, 26, 16, 
	-8, 27, 17, 
	-7, 28, 18, 
	-6, 29, 19, 
	-6, 30, 20, 
	-5, 31, 21, 
	-5, 32, 22, 
	-4, 33, 23, 
	-4, 34, 24, 
	-3, 35, 25, 
	-3, 36, 26, 
	-2, 37, 27, 
	-2, 38, 28, 
	-1, 39, 29, 
	-1, 40, 30, 
	0, 41, 31, 
};

const int8_t cos_table[] = 
{
	-126, -127, -127, -126, -123, -119, -113, -106, 
	-97, -88, -77, -66, -53, -40, -27, -13, 
	0, 14, 27, 41, 54, 66, 78, 88, 
	98, 106, 113, 119, 123, 126, 127, 127, 
	125, 122, 118, 111, 104, 95, 86, 75, 
};

const int8_t sin_table[] = 
{
	22, 8, -5, -19, -33, -46, -59, -71, 
	-82, -92, -101, -109, -116, -121, -125, -127, 
	-127, -127, -124, -121, -115, -109, -101, -92, 
	-82, -70, -58, -46, -32, -19, -5, 8, 
	22, 36, 49, 62, 73, 84, 94, 103, 
};
