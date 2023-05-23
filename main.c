/*
 * Name: Tung Nguyen
 * Real-Time Control system for Fan using Light sensor as "ground" feedback
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

enum BOOL
{
    false,
    true
};
typedef enum BOOL boolean;

enum STATES
{
    NONE,
    KEY0_ON,
    KEY0_OFF,
    KEY1_ON,
    KEY1_OFF,
    KEY2_ON,
    KEY2_OFF,
    NUM_STATES
};
typedef enum STATES State;

struct KEY_INFO
{
    unsigned int count;
    boolean state;
};
typedef struct KEY_INFO KeyInfo;

// function pointer for finite state machine
typedef State (*getNextState)(KeyInfo *keyInfo, unsigned char *timestamp);

struct PID
{
    long pGain;
    long iGain;
    long dGain;
    long last_error;
    long iState;
    char output;
};
typedef struct PID Pid;

#define LIGHT_PROCESS 120 // process light ~8x/1s - also affect how fast the fan update
#define BUTTON_PROCESS 50 // process button 20x/1s
#define ON_MAX 2
#define ON_LIMIT 2
#define OFF_MIN 1
#define OFF_LIMIT 1
#define CYCLE_STEP 5
#define SPEED0 5
#define SPEED1 100 // 100km/h
#define MAX_SPEED 255
#define MIN_SPEED 0
#define NUM_LIGHTS 100 // buffer size for averaging samples of ADC light

// PID system
#define SCALE_GAIN 1000

volatile unsigned int overflow_timer0 = 0;
volatile unsigned int overflow_timer1 = 0;
volatile unsigned char curr_duty = 0xFF; // feedback from light
volatile unsigned char curr_cmd = 0x64;  // 100km/h
volatile unsigned char fan_duty = 0xFF;
volatile long pid_output = 0xFF;
boolean high_speed = true; // initially fan run max

// heartbeat timer - 8 bit timer
void init_timer0()
{
    TCNT0 = 131;         // 125 ticks until overflow
    TCCR0B |= _BV(CS01); // prescale/8
    TIMSK0 |= _BV(TOIE0);
    overflow_timer0 = 0;
}

// button sampling rate
void init_timer1()
{
    TCNT1 = 65411;
    TCCR1B |= _BV(CS11);  // prescale/8
    TIMSK1 |= _BV(TOIE1); // enable overflow interrupt
    overflow_timer1 = 0;
}

// fan update duty cycle
void init_timer2()
{
    TCCR2A = 0x23; // clera OC2A on compare match, set at bottom (8 bit timer)
    TCCR2B = 0x01; // no prescale

    TCNT2 = 0;
    OCR2B = curr_duty;
    TIMSK2 |= _BV(TOIE2); // enable overflow interrupt
}

void sample_light()
{
    static unsigned short light[NUM_LIGHTS];
    static unsigned short light_index = 0;
    static unsigned short light_samples = 0;
    static unsigned long avg_light = 0;

    unsigned short value;
    unsigned short volts;

    // initialize the sample array...
    if (light_samples == 0)
    {
        int i;

        for (i = 0; i < NUM_LIGHTS; i++)
            light[i] = 0;
    }

    // initiate a sample
    ADCSRA |= _BV(ADSC);

    // wait for the sample to finish by checking the interrupt flag
    while (!(ADCSRA & _BV(ADIF)))
        ;
    // clear the interrupt flag for the next run
    ADCSRA |= _BV(ADIF);

    value = ADC;

    // based on 10 bits of resolution of a 2.56V reference (multiplied by 1000 for greater precision)
    volts = (value * 2560UL) / 1024UL;

    // replace oldest value with newest for the running averaging
    avg_light -= light[light_index];
    light[light_index] = volts;
    avg_light += light[light_index];

    // stop counting after we have full history
    if (light_samples < NUM_LIGHTS)
        light_samples++;

    // feedback for fan
    curr_duty = ((avg_light / light_samples) * 2 / 50); // light sensor will show from 0-102 (1100110)

    // scale the value for our 8 bits of display resolution  -- hey, 2.56V is useful!!!
    PORTB = ~((avg_light / light_samples) * 2 / 50);

    light_index = (light_index + 1) % NUM_LIGHTS;
}

// Handlers for processing state machine states
// note: not required in submitted solutions, just showing how it's done

State currStateNone(KeyInfo *keyInfo, unsigned char *timestamp)
{
    State next_state = NONE;

    (*timestamp)++;

    if (keyInfo[0].state)
    {
        next_state = KEY0_ON;
        PORTB &= ~_BV(PB0); // turn on LED 0
    }

    else if (keyInfo[1].state)
    {
        next_state = KEY1_ON;
        PORTB &= ~_BV(PB1);
    }
    else if (keyInfo[2].state)
    {
        next_state = KEY2_ON;
        PORTB &= ~_BV(PB2);
    }

    return next_state;
}

State currStateKey0On(KeyInfo *keyInfo, unsigned char *timestamp)
{
    State next_state = KEY0_ON;
    (*timestamp)++; // increment
    if (!keyInfo[0].state)
    {
        curr_cmd = SPEED0;
        next_state = KEY0_OFF;
        *timestamp = 0;
        high_speed = false;
    }
    return next_state;
}

State currStateKey0Off(KeyInfo *keyInfo, unsigned char *timestamp)
{
    PORTB |= _BV(PB0);

    return NONE;
}

State currStateKey1On(KeyInfo *keyInfo, unsigned char *timestamp)
{
    State next_state = KEY1_ON;
    (*timestamp)++; // increment
    if (!keyInfo[1].state)
    {
        curr_cmd = SPEED1;
        next_state = KEY1_OFF;
        *timestamp = 0;
        high_speed = true;
    }
    return next_state;
}

State currStateKey1Off(KeyInfo *keyInfo, unsigned char *timestamp)
{
    PORTB |= _BV(PB1);

    return NONE;
}

State currStateKey2On(KeyInfo *keyInfo, unsigned char *timestamp)
{
    State next_state = KEY2_ON;
    (*timestamp)++; // increment
    if (!keyInfo[2].state)
    {
        if (high_speed && curr_duty > 0x05)
            curr_cmd -= CYCLE_STEP;
        else if (!high_speed && curr_duty < 0xFA)
            curr_cmd += CYCLE_STEP;

        next_state = KEY2_OFF;
        *timestamp = 0;
    }
    return next_state;
}

State currStateKey2Off(KeyInfo *keyInfo, unsigned char *timestamp)
{
    PORTB |= _BV(PB2);

    return NONE;
}

void sample_inputs(KeyInfo *keyInfo)
{
    unsigned char keys = PIND;
    unsigned char key;

    for (key = PD0; key <= PD2; key++)
    {
        if ((keys & _BV(key)) == 0)
        {
            if (keyInfo[key].count < ON_MAX)
                keyInfo[key].count++;
        }
        else if (keyInfo[key].count > OFF_MIN)
            keyInfo[key].count--;

        if (keyInfo[key].count >= ON_LIMIT)
            keyInfo[key].state = true;
        else if (keyInfo[key].count <= OFF_LIMIT)
            keyInfo[key].state = false;
    }
}

// feedback for the fan's ouput - scale 1000x
long updatePID(Pid *pid, double error)
{
    // calculate Proportion
    long pTerm = pid->pGain * error;
    // calculate Integral
    pid->iState += error;
    // anti-windup 0.1*MAX/MIN value
    long iMin = (long)(255) * SCALE_GAIN / pid->iGain;
    long iMax = (long)(255) * SCALE_GAIN / pid->iGain;
    if (pid->iState > iMax)
        pid->iState = iMax;
    else if (pid->iState == iMin)
        pid->iState = iMin;
    long iTerm = pid->iGain * pid->iState;

    // calculate Derivative
    long dTerm = pid->dGain * (pid->last_error - curr_duty);
    pid->last_error = curr_duty;

    return pTerm + iTerm + dTerm;
}

// trim output range 0-255
void generateOutput(long output)
{
    output = output / SCALE_GAIN;
    if (output > MAX_SPEED)
        output = MAX_SPEED;
    else if (output < MIN_SPEED)
        output = MIN_SPEED;
    curr_duty = output; // actual update
}

int main(void)
{

    // initialize port
    // LEDs
    DDRB = 0xFF;
    PORTB = 0xFF;

    // input ADC - PF0 - light
    DDRF &= ~_BV(0);

    // output - PH6 - fan
    DDRH |= _BV(6);

    // initialize light to ADC0
    ADMUX |= _BV(REFS1) | _BV(REFS0); // 2.56V
    ADCSRA |= _BV(ADEN);              // enable ADC

    // initialize fan
    DDRH = 0xFF;
    PORTH = 0xFF;

    getNextState keyProcessing[NUM_STATES] = {currStateNone, currStateKey0On, currStateKey0Off, currStateKey1On, currStateKey1Off, currStateKey2On, currStateKey2Off};
    State curr_state = NONE;
    unsigned char timestamp = 0;

    KeyInfo keyInfo[3] = {{OFF_MIN, false}, {OFF_MIN, false}, {OFF_MIN, false}};
    Pid pid = {9500, 100, 7000, 0, 0, 0}; // scale up by 1000

    set_sleep_mode(SLEEP_MODE_IDLE);
    sei();

    init_timer0(); // light timing
    sample_light();

    init_timer2(); // pwm fan timing
    init_timer1(); // fan's hearbeat

    for (;;)
    {
        sleep_mode();
        if (overflow_timer0 >= LIGHT_PROCESS) // !should light sample quick or slow
        {
            sample_light();
            curr_state = keyProcessing[curr_state](keyInfo, &timestamp);
            pid_output = updatePID(&pid, curr_cmd - curr_duty);
            generateOutput(pid_output); // actual fan's speed update
            overflow_timer0 = 0;
        }

        if (overflow_timer1 >= BUTTON_PROCESS)
        {
            sample_inputs(keyInfo);
            overflow_timer1 = 0;
        }
    }
}
// overflow every 0.001s
ISR(TIMER0_OVF_vect)
{
    TCNT0 = 131;
    overflow_timer0++;
}

// overflow every 0.001s
ISR(TIMER1_OVF_vect)
{
    TCNT1 = 65411; // 125 ticks before overflow
    overflow_timer1++;
}

// update fan's duty cycle
ISR(TIMER2_OVF_vect)
{
    OCR2B = fan_duty;
}
