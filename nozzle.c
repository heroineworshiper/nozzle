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

// this runs on each control board
// make board0.hex
// make board1.hex
// make board2.hex
// make nozzle_fuse
// make board0_isp
// make board1_isp
// make board2_isp



#include "uart.h"
#include "nozzle.h"
#include "table.h"

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define TOGGLE_LED PORTB ^= (1 << DDB5);
#define LED_DELAY (HZ / 2)
#define LED_DELAY2 (HZ / 4)

// boundary sensor
#define HALL_THRESHOLD 16
#define HALL_CENTER 127

// ticks to wait for a motor to stop
#define MOTOR_DELAY (HZ / 2)

#if BOARD == 0
// IR code follows.  Improved version of heroineclock 2
typedef struct
{
    const uint8_t *data;
    const uint8_t value;
} ir_code_t;

// remote control codes.  Discard 1st & last symbol received since we
// only capture the rising edge.
const uint8_t up_data[] = 
{ 4,4,1,4,1,1,1,1,4,1,4,1,4,4,4,4,1,4,4,4,1,1,4,1,4,1,1,1,4,4,1,4,1 };
const uint8_t down_data[] = 
{ 4,4,1,4,1,1,1,1,4,1,4,1,4,4,4,4,1,1,4,4,1,1,4,1,4,4,1,1,4,4,1,4,1 };
const uint8_t left_data[] = 
{ 4,4,1,4,1,1,1,1,4,1,4,1,4,4,4,4,1,1,1,4,4,1,4,1,4,4,4,1,1,4,1,4,1 };
const uint8_t right_data[] = 
{ 4,4,1,4,1,1,1,1,4,1,4,1,4,4,4,4,1,4,1,4,4,1,4,1,4,1,4,1,1,4,1,4,1 };
const uint8_t fastrev_data[] = 
{ 4,4,1,4,1,1,1,1,4,1,4,1,4,4,4,4,1,4,1,1,4,4,4,4,1,1,4,4,1,1,1,1,4 };
const uint8_t fastfwd_data[] = 
{ 4,4,1,4,1,1,1,1,4,1,4,1,4,4,4,4,1,4,4,1,1,1,4,4,1,1,1,4,4,4,1,1,4 };
const uint8_t num7_data[] = 
{ 4,4,1,4,1,1,1,1,4,1,4,1,4,4,4,4,1,4,4,1,4,1,4,4,4,1,1,4,1,4,1,1,1 };
const uint8_t num8_data[] = 
{ 4,4,1,4,1,1,1,1,4,1,4,1,4,4,4,4,1,1,4,1,4,1,4,1,4,4,1,4,1,4,1,4,1 };
const uint8_t num9_data[] = 
{ 4,4,1,4,1,1,1,1,4,1,4,1,4,4,4,4,1,1,1,1,4,4,4,4,1,4,4,4,1,1,1,1,4 };
const uint8_t num0_data[] = 
{ 4,4,1,4,1,1,1,1,4,1,4,1,4,4,4,4,1,1,4,1,4,4,4,4,1,4,1,4,1,1,1,1,4 };
const uint8_t top_right_data[] =
{ 4,4,1,4,1,1,1,1,4,1,4,1,4,4,4,4,1,4,1,4,1,1,4,4,4,1,4,1,4,4,1,1,1 };
const uint8_t top_left_data[] = 
{ 4,4,1,4,1,1,1,1,4,1,4,1,4,4,4,4,1,1,1,4,1,1,4,4,4,4,4,1,4,4,1,1,1 };



#define UP 0
#define DOWN 1
#define LEFT 2
#define RIGHT 3
#define POWER 4
#define SET_PRESET 5
#define PRESET1 6
#define PRESET2 7
#define PRESET3 8
#define PRESET4 9
#define TOP_LEFT 10
#define TOP_RIGHT 11


const ir_code_t ir_codes[] = { 
    { num7_data, PRESET1 },
    { num8_data, PRESET2 },
    { num9_data, PRESET3 },
    { num0_data, PRESET4 },
    { up_data,   UP },
	{ down_data,  DOWN },
	{ left_data,  LEFT },
	{ right_data, RIGHT },
    { fastfwd_data, SET_PRESET },
    { fastrev_data, POWER },
    { top_left_data, TOP_LEFT },
    { top_right_data, TOP_RIGHT },
};

#define CODE_SIZE sizeof(up_data)
uint8_t code_buffer[CODE_SIZE];

// ticks
// 120ms observed.
#define IR_TIMEOUT (120 * HZ / 1000)
#define TOTAL_CODES (sizeof(ir_codes) / sizeof(ir_code_t))
volatile uint8_t got_ir_int = 0;
volatile uint8_t ir_time = 0;
volatile uint8_t ir_timeout = 0;
uint8_t code_offset = 0;
// IR is transmitting repeats
uint8_t have_ir = 0;
// last IR code received
uint8_t ir_code = 0;
// repeating a code or 0xff
uint8_t repeating_code = 0xff;
// delay before 1st repeat
uint8_t repeat_delay = 0;
#define REPEAT_DELAY (HZ / 2)
#define REPEAT_DELAY2 (HZ / 4)
uint8_t first_edge = 1;



typedef struct
{
    int16_t pitch;
    int16_t angle;
} preset_t;

#define PRESETS 4
#define PRESET_DELAY (HZ / 2)
uint8_t setting_preset = 0;
uint8_t current_preset = 0;
preset_t presets[PRESETS];
void (*preset_state)() = 0;
int16_t orig_nozzle_yaw = 0;
uint8_t preset_delay = 0;



// motors are armed
uint8_t armed = 0;
// blinking
uint8_t led_counter = 0;

// current state of the home operation & the motor tracking
uint8_t current_motor = 0;

void motor_idle();
void motor_home1();
void (*motor_state)() = motor_idle;


// pitch of the nozzle in polar coordinates
int16_t nozzle_pitch = 0;
// user position of motor 0 to which the nozzle_pitch adds an offset to compensate
// for the nozzle bending sideways
int16_t nozzle_yaw = 0;

// encoder step of angle adjustments
#define YAW_STEP 100



#endif // BOARD == 0


#define TOTAL_MOTORS 3
typedef struct
{
// Analog value of the hall effect sensor
    uint8_t boundary;
// encoder accumulator
    int16_t encoder;
// encoder sign.  - is towards the boundary
    int8_t sign;

// H bridge bitmasks of the 2 directions
    uint8_t dec_mask; // go towards boundary
    uint8_t inc_mask; // go away from boundary
    uint8_t total_unmask; // motor_master bits for coasting
    uint8_t total_mask; // motor_master bits for breaking

// ticks remaneing to sleep
    uint8_t timer;
    int16_t target_position;
// only move the motors if this is > 0
    uint16_t changed;
    uint8_t homing;
} tracking_state_t;
tracking_state_t tracking_state[TOTAL_MOTORS];


// raw encoder data
uint8_t prev_encoder = 0;
int8_t encoder_count = 0;

// raw boundary sensor data
uint8_t adc_downsample = 0;
#define ADC_DOWNSAMPLE 64
uint16_t adc_accum = 0;

uint8_t status_packet[4];
//uint8_t status_delay;
//uint8_t want_status = 0;
void (*receive_state)();
uint8_t receive_board = 0;
// poll every other tick
#define POLLING_DELAY 2
// test interleaved debugging output
//#define POLLING_DELAY 4
uint8_t polling_delay = POLLING_DELAY;
uint8_t polling_board = 1;
uint8_t debug_counter = 0;

#define MOTOR_TIMEOUT HZ
uint8_t motor_timeout = 0;

// state of all H bridge pins
uint8_t motor_master = 0;
#define MOTOR2_RIGHT (uint8_t)0b00100000
#define MOTOR2_LEFT  (uint8_t)0b00010000
#define MOTOR1_RIGHT (uint8_t)0b00001000
#define MOTOR1_LEFT  (uint8_t)0b00000100
#define MOTOR0_RIGHT (uint8_t)0b00000010
#define MOTOR0_LEFT  (uint8_t)0b00000001
#define TOTAL_MASK 0b00111111


#if BOARD == 0


void print_status()
{
    print_text("BOARD0: ");
    print_number(tracking_state[0].encoder);
    print_number(tracking_state[0].boundary);
    print_text("BOARD1: ");
    print_number(tracking_state[1].encoder);
    print_number(tracking_state[1].boundary);
    print_text("BOARD2: ");
    print_number(tracking_state[2].encoder);
    print_number(tracking_state[2].boundary);
    print_text("YAW=");
    print_number(nozzle_yaw);
    print_text("PITCH=");
    print_number(nozzle_pitch);
    print_text("\n");
}

// update the tracking states to get the desired nozzle pitch & roll
void update_motors()
{
    int16_t prev_positions[TOTAL_MOTORS];
    uint8_t i;

    for(i = 0; i < TOTAL_MOTORS; i++)
        prev_positions[i] = tracking_state[i].target_position;

    prev_positions[0] = tracking_state[0].target_position;
    prev_positions[1] = tracking_state[1].target_position;
    prev_positions[2] = tracking_state[2].target_position;

    CLAMP(nozzle_pitch, 0, PITCH_STEPS);
// TODO: add pitch table to get yaw encoder value
    tracking_state[0].target_position = nozzle_yaw + 
        pitch_to_encoders[nozzle_pitch * 3 + 0];
    tracking_state[1].target_position = pitch_to_encoders[nozzle_pitch * 3 + 1];
    tracking_state[2].target_position = pitch_to_encoders[nozzle_pitch * 3 + 2];

// Only move the changed motors, since the encoders aren't precise
    for(i = 0; i < TOTAL_MOTORS; i++)
    {
        if(prev_positions[i] != tracking_state[i].target_position &&
            tracking_state[i].changed < 0xff)
            tracking_state[i].changed++;
    }
}

// return 1 if the motors are tracking
uint8_t motors_tracking()
{
    if(tracking_state[0].changed > 0 ||
        tracking_state[1].changed > 0 ||
        tracking_state[2].changed > 0)
    {
        return 1;
    }
    return 0;
}

void motor_tracking()
{
// only enough clockcycles to test 1 motor per iteration
    tracking_state_t *tracking = &tracking_state[current_motor];
    uint8_t i;

    current_motor++;
    if(current_motor >= TOTAL_MOTORS)
    {
        current_motor = 0;
    }


#define YAW_REPEATING (repeating_code == LEFT || repeating_code == RIGHT)
    
// motor moving up
    if((motor_master & tracking->total_mask) == tracking->inc_mask)
    {
        if(!YAW_REPEATING && tracking->encoder >= tracking->target_position)
        {
// brake if it hit the target position & it's not an IR repeat
            motor_master |= tracking->total_mask;
        }
    }
    else
    if((motor_master & tracking->total_mask) == tracking->dec_mask)
    {
// motor moving down
        if(!YAW_REPEATING && tracking->encoder <= tracking->target_position)
        {
// brake if it hit the target position & it's not an IR command
            motor_master |= tracking->total_mask;
        }
    }
    else
// process the next command
    if(tracking->changed > 0)
    {
// print_text("changed=");
// print_number(tracking->changed);
// print_text("\n");
        if(tracking->encoder > tracking->target_position)
        {
// move motor down
            motor_master &= tracking->total_unmask;
            motor_master |= tracking->dec_mask;
            tracking->changed--;
        }
        else
        if(tracking->encoder < tracking->target_position)
        {
// move motor down
            motor_master &= tracking->total_unmask;
            motor_master |= tracking->inc_mask;
            tracking->changed--;
        }
        else
        {
// no difference in position.  Cancel the loop.
            tracking->changed--;
        }
    }
    else
// process a pitch repeat once all the motors are stopped
// the preset reader does this in preset_state, outside motor_tracking.
    if(repeating_code == UP && (motor_master & TOTAL_MASK) == TOTAL_MASK)
    {
        if(nozzle_pitch > 0)
        {
            nozzle_pitch--;
// assume the direction of the motors is 1 way & don't back up
            for(i = 0; i < TOTAL_MOTORS; i++)
            {
                int16_t new_position = pitch_to_encoders[nozzle_pitch * 3 + i];
                if(i == 0) new_position += nozzle_yaw;
                if(new_position < tracking_state[i].encoder)
                {
                    tracking_state[i].target_position = new_position;
// update the H bridge before it drifts past the target position
                    motor_master &= tracking_state[i].total_unmask;
                    motor_master |= tracking_state[i].dec_mask;
                }
            }
        }
    }
    else
    if(repeating_code == DOWN && (motor_master & TOTAL_MASK) == TOTAL_MASK)
    {
        if(nozzle_pitch < PITCH_STEPS)
        {
            nozzle_pitch++;
// assume the direction of the motors is 1 way & don't back up
            for(i = 0; i < TOTAL_MOTORS; i++)
            {
                int16_t new_position = pitch_to_encoders[nozzle_pitch * 3 + i];
                if(i == 0) new_position += nozzle_yaw;
                if(new_position > tracking_state[i].encoder)
                {
                    tracking_state[i].target_position = new_position;
// update the H bridge before it drifts past the target position
                    motor_master &= tracking_state[i].total_unmask;
                    motor_master |= tracking_state[i].inc_mask;
                }
            }
        }
    }
}


// wait a while
void motor_home6()
{
    tracking_state_t *tracking = &tracking_state[current_motor];
    if(tracking->timer == 0)
    {
        uint8_t current_encoder = tracking->encoder;

        switch(current_motor)
        {
            case 2:
                current_motor = 1;
                motor_state = motor_home1;
                break;
            case 1:
                current_motor = 0;
                motor_state = motor_home1;
                break;
            case 0:
            default:
                current_motor = 0;
                motor_state = motor_tracking;
// pitch is always 0 after homing
                nozzle_yaw = tracking->target_position;
                print_status();
                break;
        }
    }
}

// wait until home position
void motor_home5()
{
    tracking_state_t *tracking = &tracking_state[current_motor];
    if(tracking->encoder >= tracking->target_position)
    {
// brake
        motor_master |= tracking->total_mask;

        tracking->timer = MOTOR_DELAY;
        motor_state = motor_home6;
    }
}

// go until boundary is detected again
void motor_home4()
{
    tracking_state_t *tracking = &tracking_state[current_motor];
    if(ABS(tracking->boundary - HALL_CENTER) >= HALL_THRESHOLD)
    {
// reset encoder
        tracking->encoder = 0;
        motor_state = motor_home5;
    }
}

// reverse
void motor_home3()
{
    tracking_state_t *tracking = &tracking_state[current_motor];
    if(ABS(tracking->boundary - HALL_CENTER) < HALL_THRESHOLD)
    {
// always coast
        motor_master &= tracking->total_unmask;
        motor_master |= tracking->inc_mask;
        motor_state = motor_home4;
    }
}

// detect motor boundary & overshoot
void motor_home2()
{
    tracking_state_t *tracking = &tracking_state[current_motor];
    if(ABS(tracking->boundary - HALL_CENTER) >= HALL_THRESHOLD)
    {
        motor_state = motor_home3;
    }
}

// go to home position
void motor_home1()
{
    tracking_state_t *tracking = &tracking_state[current_motor];
    uint8_t sensor_value = tracking->boundary;
// sensor is already on boundary.  Abort.
// It can't arm if any sensors are on their boundary.
    if(ABS(tracking->boundary - HALL_CENTER) >= HALL_THRESHOLD)
    {
        motor_state = motor_idle;
    }
    else
// command motor to move to boundary
    {
        motor_master &= tracking->total_unmask;
        motor_master |= tracking->dec_mask;
        motor_state = motor_home2;
    }
}

void motor_home()
{
    uint8_t i;
    for(i = 0; i < TOTAL_MOTORS; i++)
    {
        tracking_state_t *tracking = &tracking_state[i];
        
        switch(tracking->homing)
        {
            case 1:
                if(ABS(tracking->boundary - HALL_CENTER) >= HALL_THRESHOLD)
                {
// go past boundary
                    tracking->homing++;
                }
                break;

            case 2:
                if(ABS(tracking->boundary - HALL_CENTER) < HALL_THRESHOLD)
                {
// coast
                    motor_master &= tracking->total_unmask;
                    tracking->timer = MOTOR_DELAY;
                    tracking->homing++;
                }
                break;

            case 3:
// find HOME
                if(tracking->timer == 0)
                {
                    motor_master |= tracking->inc_mask;
                    tracking->homing++;
                }
                break;
            
            case 4:
                if(ABS(tracking->boundary - HALL_CENTER) >= HALL_THRESHOLD)
                {
// got HOME
                    tracking->encoder = 0;
                    tracking->homing++;
                }
                break;

            case 5:
                if(tracking->encoder >= tracking->target_position)
                {
// brake
                    motor_master |= tracking->total_mask;

                    tracking->timer = MOTOR_DELAY;
                    tracking->homing++;
                }
                break;

            case 6:
// all motors finished
                if(motor_master == TOTAL_MASK)
                {
                    motor_state = motor_tracking;
// pitch is always 0 after homing
                    nozzle_yaw = tracking_state[0].target_position;
                    nozzle_pitch = 0;
                    print_status();
                }
                break;
        }
    }
}

void motor_idle()
{
}


void arm_motors()
{
    uint8_t i;
    for(i = 0; i < TOTAL_MOTORS; i++)
    {
// It can't arm if any sensors are on their boundary.
        if(ABS(tracking_state[i].boundary - HALL_CENTER) >= HALL_THRESHOLD)
        {
            return;
        }
    }


    LED_ON
    armed = 1;
    current_motor = 2;
    tracking_state[0].target_position = HOME0;
    tracking_state[1].target_position = HOME1;
    tracking_state[2].target_position = HOME2;
    tracking_state[0].changed = 0;
    tracking_state[1].changed = 0;
    tracking_state[2].changed = 0;

// command all 3 to move simultaneously.  Glitches in board 1 when horizontal
//     motor_master = tracking_state[0].dec_mask |
//         tracking_state[1].dec_mask |
//         tracking_state[2].dec_mask;
//     for(i = 0; i < TOTAL_MOTORS; i++)
//     {
//         tracking_state[i].homing = 1;
//     }
//     motor_state = motor_home;   

// move 1 at a time
    motor_state = motor_home1;
}

void disarm_motors()
{
    armed = 0;
    motor_state = motor_idle;
}

// IR interrupt
ISR(INT0_vect)
{
    got_ir_int = 1;
    ir_time = TCNT2;
    TCNT2 = 0;
}


void read_presets()
{
    uint8_t i;
    uint8_t *ptr = (uint8_t*)presets;
    print_text("read_presets: ");
    for(i = 0; i < sizeof(presets); i++)
    {
// EEPROM address
        EEARH = 0;
        EEARL = i;
// read command
        bitSet(EECR, EERE);
        *ptr++ = EEDR;
        print_hex(EEDR);
    }
    print_text("\n");
}

void write_presets()
{
    uint8_t i;
    uint8_t *ptr = (uint8_t*)presets;
    print_text("write_presets: ");
    for(i = 0; i < sizeof(presets); i++)
    {
// EEPROM address
        EEARH = 0;
        EEARL = i;
// data
        EEDR = *ptr;
// master write enable
        bitSet(EECR, EEMPE);
// write enable
        bitSet(EECR, EEPE);
        print_hex(*ptr);
        ptr++;
// wait for it
        while(bitRead(EECR, EEPE))
        {
        }
    }
    print_text("\n");
}


void do_preset_save2()
{
    if(preset_delay == 0)
    {
        nozzle_yaw = orig_nozzle_yaw;
        update_motors();
        preset_state = 0;
    }
}

void do_preset_save1()
{
    if(!motors_tracking())
    {
        preset_delay = PRESET_DELAY;
        preset_state = do_preset_save2;
    }
}

void do_preset_pitch()
{
    if(!motors_tracking())
    {
        if(nozzle_pitch < presets[current_preset].pitch)
        {
            nozzle_pitch++;
            update_motors();
        }
        else
        {
// done
            preset_state = 0;
            LED_ON
        }
    }
}

void do_preset_yaw()
{
    if(!motors_tracking())
    {
        nozzle_yaw = presets[current_preset].angle;
        update_motors();
        preset_state = do_preset_pitch;
    }
}

void do_preset_center()
{
// wait for motors to finish
    if(!motors_tracking())
    {
// go to preset in 1 motion
//         nozzle_yaw = presets[current_preset].angle;
//         nozzle_pitch = presets[current_preset].pitch;
//         update_motors();
//         preset_state = 0;
// center it 1 step at a time
        if(nozzle_pitch > 0)
        {
            nozzle_pitch--;
            update_motors();
        }
        else
            preset_state = do_preset_yaw;
    }
}


void handle_preset_button(int preset)
{
    if(setting_preset)
    {
        presets[preset].pitch = nozzle_pitch;
        presets[preset].angle = nozzle_yaw;
        write_presets();
        setting_preset = 0;
        LED_ON

// wiggle the nozzle
        orig_nozzle_yaw = nozzle_yaw;
        nozzle_yaw += YAW_STEP;
        update_motors();
        preset_state = do_preset_save1;
    }
    else
    {
        current_preset = preset;
        if(presets[preset].angle != 0xff &&
            presets[preset].pitch != 0xff)
        {
// center it
            preset_state = do_preset_center;
        }
    }
}


void handle_ir_code()
{
    uint8_t i;

//    print_text("IR=");
//    print_number(ir_code);
//    print_text("\n");
	switch(ir_code)
	{
        case UP:
            setting_preset = 0;
            preset_state = 0;
            TOGGLE_LED

            if(armed)
            {
                if(repeating_code == 0xff && nozzle_pitch > 0)
                {
                    nozzle_pitch--;
                    update_motors();
                }
            }
            else
            {
// test motor
                motor_master = MOTOR1_RIGHT;
            }
            break;

        case DOWN:
            setting_preset = 0;
            preset_state = 0;
            TOGGLE_LED

            if(armed)
            {
                if(repeating_code == 0xff && nozzle_pitch < PITCH_STEPS)
                {
                    nozzle_pitch++;
                    update_motors();
                }
            }
            else
            {
// test motor
                motor_master = MOTOR1_LEFT;
            }
            break;

        case LEFT:
            setting_preset = 0;
            preset_state = 0;
            TOGGLE_LED

            if(armed)
            {
                if(repeating_code == 0xff)
                {
// move 1 step
                    nozzle_yaw += YAW_STEP;
                    update_motors();
                }
                else
                {
// move motor continuously
                    motor_master = MOTOR0_RIGHT;
                }
            }
            else
            {
// test motor
                motor_master = MOTOR0_RIGHT;
            }
            break;

        case RIGHT:
            setting_preset = 0;
            preset_state = 0;
            TOGGLE_LED


            if(armed)
            {
                if(repeating_code == 0xff)
                {
                    nozzle_yaw -= YAW_STEP;
                    update_motors();
                }
                else
                {
// move motor continuously
                    motor_master = MOTOR0_LEFT;
                }
            }
            else
            {
// test motor
                motor_master = MOTOR0_LEFT;
            }
            break;

        case TOP_LEFT:
            setting_preset = 0;
            preset_state = 0;
            TOGGLE_LED
            
            if(!armed)
            {
// test motor
                motor_master = MOTOR2_RIGHT;
            }
            break;

        case TOP_RIGHT:
            setting_preset = 0;
            preset_state = 0;
            TOGGLE_LED

            if(!armed)
            {
// test motor
                motor_master = MOTOR2_LEFT;
            }
            break;

// no repeat
        case SET_PRESET:
            if(setting_preset)
            {
                setting_preset = 0;
            }
            else
            if(armed)
            {
                setting_preset = 1;
            }
            break;

        case PRESET1:
            handle_preset_button(0);
            break;

        case PRESET2:
            handle_preset_button(1);
            break;

        case PRESET3:
            handle_preset_button(2);
            break;

        case PRESET4:
            handle_preset_button(3);
            break;

// no repeat
        case POWER:
            setting_preset = 0;
            preset_state = 0;
            TOGGLE_LED
            if(!armed)
            {
                arm_motors();
            }
            else
            {
                disarm_motors();
            }
            break;
	}
}

void handle_ir()
{
    uint8_t i;
// IR timed out
    if(ir_timeout == 0)
    {
		code_offset = 0;


        if(have_ir)
        {
// get latest positions to stop continuous motion
            if(armed && repeating_code != 0xff)
            {
                for(i = 0; i < TOTAL_MOTORS; i++)
                {
                    tracking_state[i].target_position = 
                        tracking_state[i].encoder;
                }

// subtract pitch table value to get yaw
                nozzle_yaw = tracking_state[0].encoder - 
                    pitch_to_encoders[nozzle_pitch * 3 + 0];
// nozzle_pitch is updated in handle_tracking
                print_status();
            }
            else
                print_status();

// stop LED
            if(armed && 
                !setting_preset &&
                preset_state == 0)
                LED_ON

// stop motors
            if(!armed)
            {
                motor_master = 0;
            }

    		have_ir = 0;
            repeating_code = 0xff;
        }
    }



// repeat IR code after 1st delay & motors are all in braking mode
    if(have_ir && 
        repeat_delay == 0 &&
        ir_code != POWER &&
        ir_code != SET_PRESET)
    {
//print_text("IR repeat\n");
        repeat_delay = REPEAT_DELAY2;
        if(armed) repeating_code = ir_code;
        handle_ir_code();
    }


    if(got_ir_int)
    {
        got_ir_int = 0;
// 5 total symbols observed but we only use 2

//        if(ir_time > 117) 
//            ir_time = 4;
//        else
//        if(ir_time > 62) 
//            ir_time = 3;
//        else
//        if(ir_time > 55) 
//            ir_time = 2;
//        else
//        if(ir_time > 22) 
//            ir_time = 1;
//        else
//            ir_time = 0;


        if(ir_time > 55)
            ir_time = 4;
        else
            ir_time = 1;

//print_number(ir_time);
        ir_timeout = IR_TIMEOUT;
        
        
        if(!have_ir)
        {
            code_buffer[code_offset++] = ir_time;
            if(code_offset >= CODE_SIZE)
            {
// end of code
// search for the code
	            uint8_t i, j;
// got complete code
	            uint8_t got_it = 0;
	            for(i = 0; i < TOTAL_CODES && !got_it; i++)
//	            for(i = 0; i < 5 && !got_it; i++)
                {
                    const ir_code_t *code = &ir_codes[i];
                    const uint8_t *data = code->data;
                    got_it = 1;
// ignore 1st byte
                    for(j = 1; j < CODE_SIZE; j++)
                    {
// print_number(data[j]);
// print_text("=");
// print_number(code_buffer[j]);
// print_text("\n");
                        if(data[j - 1] != code_buffer[j])
                        {
                            got_it = 0;
                            break;
                        }
                    }

                    if(got_it)
                    {
// print_text("i=");
// print_number(i);
// print_text("\n");
					    have_ir = 1;
                        repeat_delay = REPEAT_DELAY;
					    ir_code = code->value;
                        handle_ir_code();
                    }
                }

                if(!got_it)
                {
                    code_offset = 0;
                }
            }
        }
    }
}
#endif // BOARD 0



void get_code1();

#if BOARD != 0
void get_code3()
{
    receive_state = get_code1;
    motor_master = uart_in;
    motor_timeout = MOTOR_TIMEOUT;
// construct status packet
    status_packet[0] = 0xff;
    status_packet[1] = 0x54 | BOARD;
    status_packet[2] = tracking_state[BOARD].boundary;
    status_packet[3] = encoder_count;
    encoder_count = 0;
    send_uart(status_packet, 4);
// delay to let the UART switch directions
//    status_delay = 0;
//    want_status = 1;
}

void get_code2()
{
// polling request
    if(uart_in == (0xa8 | BOARD))
        receive_state = get_code3;
    else
    if(uart_in == 0xff)
        receive_state = get_code2;
    else
        receive_state = get_code1;
}
#else // BOARD != 0

void get_code4()
{
// encoder count
    int8_t encoder_increment = uart_in;
    tracking_state[receive_board].encoder += 
        encoder_increment * tracking_state[receive_board].sign;
    receive_state = get_code1;
}

void get_code3()
{
// ADC result
    tracking_state[receive_board].boundary = uart_in;
    receive_state = get_code4;
}

void get_code2()
{
// response to polling
    if((uart_in & 0x54) == 0x54 &&
        (uart_in & 0x3) < 3)
    {
        receive_state = get_code3;
        receive_board = uart_in & 0x3;
    }
    else
    if(uart_in == 0xff)
        receive_state = get_code2;
    else
        receive_state = get_code1;
}




#endif // BOARD == 0

void get_code1()
{
    if(uart_in == 0xff)
        receive_state = get_code2;
}

void main()
{
    uint8_t i;

// disable watchdog
	WDTCSR = 0;

	init_serial();
#if BOARD == 0
	print_text("\n\nWelcome to F-35 nozzle\n");
#endif

#if BOARD == 1
	print_text("\n\nWelcome to board 1\n");
#endif

#if BOARD == 2
	print_text("\n\nWelcome to board 2\n");
#endif

// motor H bridge
    bitClear(PORTD, 3);
    bitClear(PORTD, 4);
    bitSet(DDRD, 3);
    bitSet(DDRD, 4);


// LED/DEBUG
    DDRB |= 1 << DDB5;
    LED_ON

// tick clock prescaler page 108
// CLKio is 32khz
    TCCR0B = 0b00000100;

#if BOARD == 0


// enable IR interrupt page 72
    EIMSK = 0b00000001;
// rising edge interrupt page 71
    EICRA = 0b00000011;
// enable IR timer page 156
    TCCR2B = 0b00000100;

// initialize motor tables
    for(i = 0; i < TOTAL_MOTORS; i++)
    {
        tracking_state[i].boundary = HALL_CENTER;
        tracking_state[i].encoder = 0;
    }

    tracking_state[2].dec_mask = MOTOR2_LEFT;
    tracking_state[2].inc_mask = MOTOR2_RIGHT;
    tracking_state[2].total_unmask = ~(MOTOR2_LEFT | MOTOR2_RIGHT);
    tracking_state[2].total_mask = MOTOR2_LEFT | MOTOR2_RIGHT;
    tracking_state[2].sign = 1;

    tracking_state[1].dec_mask = MOTOR1_RIGHT;
    tracking_state[1].inc_mask = MOTOR1_LEFT;
    tracking_state[1].total_unmask = ~(MOTOR1_LEFT | MOTOR1_RIGHT);
    tracking_state[1].total_mask = MOTOR1_LEFT | MOTOR1_RIGHT;
    tracking_state[1].sign = -1;

    tracking_state[0].dec_mask = MOTOR0_LEFT;
    tracking_state[0].inc_mask = MOTOR0_RIGHT;
    tracking_state[0].total_unmask = ~(MOTOR0_LEFT | MOTOR0_RIGHT);
    tracking_state[0].total_mask = MOTOR0_LEFT | MOTOR0_RIGHT;
    tracking_state[0].sign = 1;

#endif // BOARD == 0

// boundary sensor page 249
    ADMUX = 0b01000000;
    ADCSRA = 0b10000111;
    bitSet(ADCSRA, ADSC);


    receive_state = get_code1;

#if BOARD == 0
    read_presets();
#endif

// enable interrupts
	sei();


// delay before polling
#if BOARD == 0
    uint16_t delay = HZ;
#endif

// delay for each board's print
#if BOARD == 1
    uint16_t delay = HZ / 2;
#endif

#if BOARD == 2
    uint16_t delay = 0;
#endif

    while(delay > 0)
    {
// handle tick
        if(bitRead(TIFR0, TOV0))
        {
            bitSet(TIFR0, TOV0);
            delay--;
        }
    }


	while(1)
	{
		handle_serial();
		if(have_uart_in)
		{
			have_uart_in = 0;

            receive_state();
		}

#if BOARD == 0
// handle tick
        if(bitRead(TIFR0, TOV0))
        {
            bitSet(TIFR0, TOV0);
// DEBUG
//            TOGGLE_LED

// update motor timers
            for(i = 0; i < TOTAL_MOTORS; i++)
            {
                tracking_state_t *tracking = &tracking_state[i];
                if(tracking->timer > 0)
                {
                    tracking->timer--;
                }
            }

            polling_delay--;
            if(polling_delay == 0)
            {
                polling_delay = POLLING_DELAY;
                uint8_t polling_packet[3];
                polling_packet[0] = 0xff;
                polling_packet[1] = 0xa8 | polling_board;
                polling_packet[2] = motor_master;
                send_uart(polling_packet, 3);


                if(polling_board == 2)
                    polling_board = 1;
                else
                    polling_board = 2;
            }

// service the debug output in every tick we're not polling
            if(polling_delay != POLLING_DELAY)
            {
                debug_counter++;
                if(debug_counter >= 10)
                {
                    debug_counter = 0;
// print the status
                    print_status();
                }
                handle_debug();
            }

            if(ir_timeout > 0) ir_timeout--;


            if(!armed)
            {
                led_counter++;
                if(led_counter >= LED_DELAY)
                {
                    led_counter = 0;
                    
// toggle it if unarmed & no motors are being tested
                    if(motor_master == 0)
                        TOGGLE_LED
                }
            }
            else
            if(setting_preset || preset_state != 0)
            {
                led_counter++;
                if(led_counter >= LED_DELAY2)
                {
                    led_counter = 0;
                    TOGGLE_LED
                }
            }


            if(repeat_delay > 0) repeat_delay--;
            if(preset_delay > 0) preset_delay--;
        }

        motor_state();

        handle_ir();

        if(preset_state != 0)
            preset_state();
#else // BOARD 0



// handle tick
        if(bitRead(TIFR0, TOV0))
        {
            bitSet(TIFR0, TOV0);

            handle_debug();

// shut down if no packet received
            if(motor_timeout == 0)
                motor_master = 0;
            else
                motor_timeout--;

//             if(want_status)
//             {
//                 if(status_delay > 0)
//                     status_delay--;
//                 else
//                 {
//                     want_status = 0;
//                     send_uart(status_packet, 4);
//                 }
//             }
        }
#endif // BOARD != 0


        uint8_t current_encoder = PINC & 0b00001100;
        if(current_encoder != prev_encoder)
        {
            if(current_encoder == 0b00001000 &&
                prev_encoder == 0b00001100)
            {
                encoder_count++;
            }
            else
            if(current_encoder == 0b00001100 &&
                prev_encoder == 0b00001000)
            {
                encoder_count--;
            }
            prev_encoder = current_encoder;

#if BOARD == 0
            tracking_state[0].encoder += encoder_count * tracking_state[0].sign;
            encoder_count = 0;
#endif

//             print_number(bitRead(PINC, 2));
//             print_number(bitRead(PINC, 3));
//             print_number(encoder_count);
//             print_text("\n");
        }


// boundary sensor        
        if(bitRead(ADCSRA, ADIF))
        {
            bitSet(ADCSRA, ADIF);
// must read the right byte order to update the registers
            uint8_t low = ADCL;
            uint8_t high = ADCH;
            adc_accum += (high << 8) | low;
            adc_downsample++;
            if(adc_downsample >= ADC_DOWNSAMPLE)
            {
//print_number(adc_accum / ADC_DOWNSAMPLE);
//print_text("\n");
                tracking_state[BOARD].boundary = adc_accum / ADC_DOWNSAMPLE / 4;
                adc_accum = 0;
                adc_downsample = 0;
            }
            bitSet(ADCSRA, ADSC);
        }


// update motors
#if BOARD == 0
        bitWrite(PORTD, 3, (motor_master & MOTOR0_LEFT));
        bitWrite(PORTD, 4, (motor_master & MOTOR0_RIGHT));
#endif

#if BOARD == 1
        bitWrite(PORTD, 3, (motor_master & MOTOR1_LEFT));
        bitWrite(PORTD, 4, (motor_master & MOTOR1_RIGHT));
#endif

#if BOARD == 2
        bitWrite(PORTD, 3, (motor_master & MOTOR2_LEFT));
        bitWrite(PORTD, 4, (motor_master & MOTOR2_RIGHT));
#endif
    }
}








