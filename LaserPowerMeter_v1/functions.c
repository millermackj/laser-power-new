/*******************************************************
Name: functions.c
Date: 11/13/2012
Author: Jacob Miller-Mack
Comments:  Support file for functions
 *******************************************************/
#include "p33FJ64MC202.h"   // Include p33FJ64MC202 header file
#include "math.h"           // Include math libary
#include "support.h"
#include "PIC_serial.h"
//#include "PID.h"     // Include Defitions and function prototypes
#include <stdio.h>          // Include C standard I/O library
#include <stdlib.h>         // include C standard library
#include <string.h>         // for work with strings

extern long unsigned int run_time;  // run time in milliseconds
extern int use_mm_pos;
extern int wait_flag;     // signals end of sample period
extern int sample_time;
extern int post_period;
extern int blink_period;
extern int step_period;
extern motor_struct motorX, motorY;
extern command_struct command;
extern int pause_all;
extern int printHeader;
extern int sys_state;
extern int testing;
extern int postflag;
char toPrint2[BUFFER_SIZE];
extern int post_period;
extern int blink_period;
extern int max_travel;
extern gradient_data_struct quadrant[];
extern int use_simple_filter; // flag to use EWMA instead of butterworth filter
extern int inch_to_mm_scale;
extern int mm_to_inch_scale;
extern int need_user_confirm;

extern int quarter_clock; // 500 usec timer

// Initialize of Clock Frequency

//Configuration Bits for Clock Initialization Parameters
_FOSCSEL(FNOSC_FRC); // Select Internal FRC at POR
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON); // Enable Clock Switching and use OSC2 as i/0
_FWDT(FWDTEN_OFF); // Turn off Watchdog Timer
_FICD(JTAGEN_OFF & ICS_PGD1); // Turn off JTAG and communicate on PGC1/EMUC1 and PGD1/EMUD1

void init_clock(void) {
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    // 7.37 MHz * 43 / 2 / 2 = 79.227 MHz
    PLLFBD = 41; // M = 43
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    CLKDIVbits.PLLPRE = 0; // N1 = 2

    // Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(0x01);

    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b001);

    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1) {
    };
} //end init_clock

//////////////////////////////////////////////////////////
// Timer1 interupt service routine
// Used to enforce Sample Time

void _ISRFASTER _T1Interrupt(void) {

  quarter_clock++; // increment 500 usec timer
  if (quarter_clock % 4 == 0){
     run_time++; // increment millisecond timer
     quarter_clock = 0;
  }
  run_steppers(quarter_clock % 2); //

//  if(quarter_clock % 2 == 0)
//    run_steppers(quarter_clock / 2);

  if (run_time % sample_time == 0)
      wait_flag = 1; // Signal end of sample time
  
  IFS0bits.T1IF = 0;
} // end T1Interupt

// Timer2 iterrupt service routine
// used to update pwm1 and pwm2 duty cycles
void _ISRFASTER _T2Interrupt(void) {
//    OC1RS = motorX.pwm.duty;
//    OC1R = PWM_PERIOD - motorX.pwm.duty;
//
//    OC2RS = motorY.pwm.duty;
//    OC2R = PWM_PERIOD - motorY.pwm.duty;
//
//    OC3RS = coilPWM;
//    OC3R = PWM_PERIOD - coilPWM;

    IFS0bits.T2IF = 0; // reset interrupt flag
} // end T2Interupt

//////////////////////////////////////////////////////////
// Intialize Sample Time (1 msec)

void init_samptime(void) {
    TMR1 = 0;
//    PR1 = 4952 - 1; // Set the period register - 1 msec
    PR1 = 1238 - 1; // Set the period register - 500 usec

    T1CON = 0x8010; // TMR1 on, prescalar set to 1:8 Tclk/2
    _T1IF = 0; // Clear Flag
    _T1IE = 1; // Enable Source
} //end init_samptime


//////////////////////////////////////////////////////////
//Initialize Pins

void init_pins(void){
/* Pin Overview Map
          NEW:
                RB0 - thermopile quad0  RA0 - Potentiometer
                RB1 - thermopile quad1  RA1 - thermocouple in absorber
                RB2 - thermopile quad2  RA2 - GRN_LED
                RP3 - thermopile quad3  RA3 - RED_LED
                RP4 - UART1 TX          RA4 - Button
                RP5 - UART1 RX
                RP6 - Enc1 phaseA
                RP7 - Enc1 phaseB
                RP8 - Enc2 phaseA
                RP9 - Enc2 phaseB
                RP10 - MOTY_EN
                RP11 - MOTY_DIR
                RB12 - MOTY_STEP
                RB13 - MOTX_EN
                RB14 - MOTX_DIR
                RB15 - MOTX_STEP
  */
    AD1PCFGL = 0b1111111111000000;  // AN0-5 analog inputs
    //      0b5432109876543210
    TRISA = 0b0000000000010011; // RA0,1,4 input, rest PORTA pins output
    ODCA = 0;           // all PORTA pins set NOT open drain
    LATA = 0x0000;      // all PORTA output pins set to zero
    //      0b5432109876543210
    TRISB = 0b0000001111101111; // RB0-3,5-9, input,rest PORTB pins set to output
    ODCB = 0x0000;  //  no open drain pins on PORTB
    LATB = 0x0000;  //  all PORTB output pins set to zero

    // encoder pin assignment
    RPINR14bits.QEA1R = 6; // encoder1 phase A on RP6
    RPINR14bits.QEB1R = 7; // encoder1 phase B on RP7
    RPINR16bits.QEA2R = 9; // encoder2 phase A on RP9
    RPINR16bits.QEB2R = 8; // encoder2 phase B on RP8

    // pwm pin assignments
//    RPOR5bits.RP10R = 0b10010; // RP10 to output compare 1 (pwm1)
//    RPOR6bits.RP13R = 0b10011; // RP13 to output compare 2 (pwm2)
//    RPOR2bits.RP5R =  0b10100; // RP5 to output compare 3(voice coil pwm)

    // init voice coil PWM
    OC3CON = 0x0000; // disable PWM module
//    OC3RS = coilPWM; // set slave duty cycle
//    OC3R = PWM_PERIOD - coilPWM; // duty cycle
//    OC3CON = 0x0006; // activate PWM module

    // UART1 pin assignment
    RPINR18bits.U1RXR = 5;  // UART1 RX set to RP5
    RPOR2bits.RP4R = 0x3;   // RP4 set to UART1 TX

} // end init_pins

/*  button debouncing routine for polling button pin each sample period
    returns BTN_OFF unless button has just been let up.
    otherwise button has just been released:
    returns BTN_SHORT if button has been pressed more than BTN_THRESHOLD_LOW ms
    returns BTN_LONG if button has been pressed more than BTN_THRESHOLD_HI ms
 */
int btnDebounce(void) {
    // shared variables among all debounce calls

    // pushbutton state: off, short press, or long press
    static int button_state = BTN_OFF;

    // button debouncing counter gets incremented each ms button is down
    static int button_cts = 0;

    // check if button pin is high
    if (BTN_PORT & BTN_PIN) {
        button_cts++; // if pin's high, increment button count
    } else if (button_cts > 0) {
        button_cts--; // if button pin is low, decrement button count until zero
    }

    // decide what to do if button is not being pressed right now
    if (button_cts == 0) {
        switch (button_state) {
            case BTN_LONG: // button has been released after long press
                button_state = BTN_OFF; // reset button state to off
                return BTN_OFF;
            case BTN_SHORT: // button has been released after short press
                button_state = BTN_OFF; // reset button state
                return BTN_SHORT;
            default: // button has not been pressed at all
                return BTN_OFF; // return 0
        }
    }  // set button state if it has newly reached the short press threshold
    else if (button_cts == BTN_THRESHOLD_LOW && button_state == BTN_OFF) {
        button_state = BTN_SHORT; // set button state as short press
    } // set state to long press if it has newly reached long press threshold
    else if (button_cts == BTN_THRESHOLD_HI && button_state == BTN_SHORT) {
        button_state = BTN_OFF; // set button state to long press
        return BTN_LONG;
    }

    return BTN_OFF;
} // end btnDebounce

// initialize analog to digital converter

void init_ADC(void) {

    AD1CON1 = 0x04e0; // 12-bit conversion, auto convert after sampling
    AD1CSSL = 0; // disable scanning
    AD1CON2 = 0; // use MUXA
    AD1CON3 = 0x1f02; // max sample time = 31 * a/d clocktime
    
    AD1CON1bits.SIMSAM = 0; // sample one channel at a time
    AD1CON1bits.ADON = 1; // turn on ADC module
} // end init_ADC

// initialize quadrature encoder module
void init_encoders(enc_struct* encoder1, enc_struct* encoder2){
    /* possible index_mod e settings:
    111 = Quad Enc Interface enabled (x4 mode) position counter reset by (MAXxCNT)
    110 = Quad Enc Interface enabled (x4 mode) Index Pulse reset of position ctr
    101 = Quad Enc Interface enabled (x2 mode) position counter reset by (MAXxCNT)
    100 = Quad Enc Interface enabled (x2 mode) Index Pulse reset of position ctr
    011 = Unused (Module disabled)
    010 = Unused (Module disabled)
    001 = Starts 16-bit Timer
    000 = Quadrature Encoder Interface/Timer off
     */
    //  QEI1CONbits.QEISIDL = 0;  // continue module operation in even in idle mode
    //
    //  QEI1CONbits.QEIM = index_mode; // set interface mode according to index_mode
    //
    //  QEI1CONbits.SWPAB   = 0;  // don't swap A and B phases
    //  QEI1CONbits.PCDOUT  = 0;  // disable position counter status output
    //  QEI1CONbits.POSRES  = 0;  // don't reset position ctr with index pulse
    //
    //  DFLT1CONbits.IMV    = 0;  // index match value (??)
    //  DFLT1CONbits.CEID   = 1;  // disable interrupts due to count errors
    //  DFLT1CONbits.QEOUT  = 0;  // disable digital filter output
    //  DFLT1CONbits.QECK   = 3;  // set clock subdivision 1:16

    QEI1CON = 0x0700; // initialize encoder interface
    MAX1CNT = ONE_REV; // number of counts per encoder revolution
    POS1CNT = 0; // reset encoder counts

    QEI2CON = 0x0700; // initialize encoder interface
    MAX2CNT = ONE_REV; // number of counts per encoder revolution
    POS2CNT = 0; // reset encoder counts

    // set up data structure for encoder1
    encoder1->POSCOUNT = &POS1CNT; // register pointer for encoder1
    // initialize variables to zero
    encoder1->posn_ref = 0;
    encoder1->posn_count = 0;
    encoder1->posn_delta = 0;
    encoder1->posn_old = 0;
    encoder1->posn_countold = 0;
    encoder1->posn = 0;

    // set up data structure for encoder2
    encoder2->POSCOUNT = &POS2CNT; // register pointer for encoder2
    // initialize variables to zero
    encoder2->posn_ref = 0;
    encoder2->posn_count = 0;
    encoder2->posn_delta = 0;
    encoder2->posn_old = 0;
    encoder2->posn_countold = 0;
    encoder2->posn = 0;
} // end init_Encoder

// function to read a/d channel. returns 12-bit (0 to 4096) voltage reading.
long int read_ADC(int channel) {
    int i = 0; // counter for waiting loop
    AD1CHS0 = channel; // indicate which pin is to be read
    while(i<25) i++;
    AD1CON1bits.SAMP = 1; // begin sampling
    while (i >0)i--; // twiddle our thumbs for a couple instruction cycles
    while (!AD1CON1bits.DONE); // wait for sampling to be done
    return (long int)ADC1BUF0; // return the a/d results
} // end read_ADC

// function to get encoder data for the specified encoder
// takes pointer to position data strutcure

void readEncoder(enc_struct* pos_data) {
    // Capture the current count from position register
    pos_data->posn_count = *(pos_data->POSCOUNT)/2;

    // calc the difference between samples
    pos_data->posn_delta = pos_data->posn_count - pos_data->posn_countold;

    // Detect and correct for zero crossing, will be apparent with large deltas.
    // If the difference is less than -50 (bigger than expected), then correct
    if (pos_data->posn_delta < -50) {
        pos_data->posn = pos_data->posn_old + pos_data->posn_delta + ONE_REV;
    }// If delta is greater than 50 (bigger than expected), correct it
    else if (pos_data->posn_delta > 50) {
        pos_data->posn = pos_data->posn_old + pos_data->posn_delta - ONE_REV;
    }// if delta is small, than just update position normally with delta.
    else {
        pos_data->posn = pos_data->posn_old + pos_data->posn_delta;
    }

    // update old values.
    pos_data->posn_countold = pos_data->posn_count;
    pos_data->posn_old = pos_data->posn;

}

// function to update the digital output states of the led pins
// takes pointer to LED structure
void updateLEDs(LED_struct* LED){
  // read contents of LATB and create mask with led pins cleared
  unsigned int tempLATCH = *(LED->LATCH) & ~(GRN_LED | RED_LED);
  // set LED pins correctly on LATB
  *(LED->LATCH) = tempLATCH | *(LED->grn)*GRN_LED | *(LED->red)*RED_LED;
}

//send a step to a stepper motor driver
void step_on(motor_struct* motor, int direction){
  unsigned int tempLATCH = *(motor->LATCH) & ~(motor->DIR_PIN | motor->STEP_PIN);
  *(motor->LATCH) = tempLATCH | motor->DIR_PIN*direction | motor->STEP_PIN;
}

// remove pulse from stepper motor driver
void step_off(motor_struct* motor){
  *(motor->LATCH) = *(motor->LATCH) & ~(motor->STEP_PIN);
}

// enable or disable stepper motor
void motor_enable(motor_struct* motor, int enable){
    unsigned int tempLATCH = *(motor->LATCH) & ~(motor->ENABLE_PIN);

    *(motor->LATCH) = tempLATCH | (motor->ENABLE_PIN*!enable);
 
  motor->enabled = enable; // set enabled status of motor

}

// run the default filter on the data
long int filter(gradient_data_struct* data, long int new_input, int doOffset) {
  data->unfiltered_value = new_input; // keep a copy of the current raw value

  data->inputs_head = data->inputs_head->prev;
  data->inputs_head->datum = new_input;
  
  if (doOffset) {
    new_input = (((new_input- data->offset) * data->scaling_factor) >> 15) ;
    //new_input -= data->offset;
  }

  // move the pointer backwards in past outputs list
  data->outputs_head = data->outputs_head->prev;
  // at the end of method, outputs_head will point to most recent value

  if (use_simple_filter) {
    // output = k*(input) + (1-k)*last_output
    data->outputs_head->datum = (new_input * data->k1
            + data->outputs_head->next->datum * data->k2) / 1000; //
    
  } else {
    data->filtered_value = 0; // start anew
    // place new input in the array of past values
    data->inputs_head = data->inputs_head->prev;
    data->inputs_head->datum = new_input;
    int i;

    // multiply each of the past inputs by their filter coefficients
    list_element* iter = data->inputs_head;
    for (i = 0; i < data->filter->filter_order + 1; i++) {
      data->outputs_head->datum += (iter->datum * data->filter->input_coeffs[i]) >> 17;
      iter = iter->next;
    }

    // multiply each of the past outputs by their filter coefficients
    iter = data->outputs_head;
    for (i = 1; i < data->filter->filter_order + 1; i++) {
      data->filtered_value -= (iter->datum * data->filter->output_coeffs[i]) >> 17;
      iter = iter->next;
    }
  }

  data->filtered_value = data->outputs_head->datum;
  return data->filtered_value; // return the filtered value
}
// obtain derivative of filtered signal.
// average first half of past values, then average second half
// derivative is (ave_2-ave_1)/delta_t, where delta_t is half of time history
long int differentiate(gradient_data_struct* data){
  int i;
  long int sum = 0;
  list_element* iter = data->outputs_head;
  for(i = 0; i < NUM_TAPS/2; i++){
    sum += iter->datum;
    iter = iter->next;
  }

  for(i = NUM_TAPS/2;i < NUM_TAPS; i++){
    sum -= iter->datum;
    iter = iter->next;
  }

  data->deriv = sum * (1000/AD_PERIOD)/(NUM_TAPS/2 * NUM_TAPS/2);
  return data->deriv;
}

// obtain the (unit time step) integral of a signal
long unsigned int integrate(gradient_data_struct* data){
  return data->integral += data->filtered_value;
}

void init_filter(digital_filter* filter, int filter_order){
  filter->filter_order = filter_order;
}

// calculate average of stored past values
long int get_average(gradient_data_struct* data){
  int i = 0; // a counter
  data->average = 0;
  list_element* iter = data->inputs_head; // list iterator
  // sum up each of the stored inputs
  do{
    data->average += iter->datum;
    iter = iter->next;
    i++; // count up each stored input
  }while(iter != data->inputs_head);

  data->average /= i; // calculate mean value of inputs

  return data->average;
}

// convert integer deltas into a fixed point scaling factor
void calc_scale(gradient_data_struct* data, long int delta_Y, long int delta_X) {
  data->dbl_scale = (double)delta_Y / (double)delta_X;
  double scale = data->dbl_scale * powf(2, 15);
  if (fmod(scale, 1.0) >= 0.5)
    scale += 1.0;
  data->scaling_factor = (long int)(scale);
}

// convert floating point scaling factor into fixed point scaling factor
void calc_scale_dbl(gradient_data_struct* data, double dbl_scale){
  data->dbl_scale = dbl_scale;
  double scale = (dbl_scale) * powf(2, 15);
  if (fmod(scale, 1.0) >= 0.5)
    scale += 1.0;
  data->scaling_factor = (long int)(scale);
}

void calc_offsets(gradient_data_struct (*data)[]){
  // calculate average value of the data stream
  int i;
  for(i = 0; i < 4; i++){
    get_average(&((*data)[i]));
    (*data)[i].offset = (*data)[i].average;
  }
}


// calculate the scaling offsets for each data stream to read zero at cold state
void calc_cold_offsets(gradient_data_struct (*data)[]){
//  // find minimum offset
  int min_index = 0;
  get_average(&((*data)[0]));

  int i;
  // calculate averages and take note of minimum
  for (i = 1; i < 4; i++){
    get_average(&((*data)[i]));
    if ((*data)[i].average < (*data)[min_index].average)
      min_index = i;
  }

  // determine scaling factors
  for(i = 0; i < 4; i ++){
    double num, den;
    // set offset to be the average of the lowest-reading output
    (*data)[i].offset = (*data)[min_index].average;
    num = (double) (*data)[min_index].average;
    den = (double) (*data)[i].average;
    // scale each quadrant down to minimum average
    calc_scale_dbl(&((*data)[i]), (num/den));

//    scale = (num / den) * powf(2,15);
//    if (fmod(scale, 1.0) >= 0.5)
//      scale += 1.0;
//    (*data)[i].scaling_factor = (long int)(scale);
  }

  sprintf(toPrint2, "<m>calibrating...\naverage/scaling:\na: %ld/%ld\nb: %ld/%ld\nc: %ld/%ld\nd: %ld/%ld</m>\n",
          (*data)[0].average, (*data)[0].scaling_factor,
          (*data)[1].average, (*data)[1].scaling_factor,
          (*data)[2].average, (*data)[2].scaling_factor,
          (*data)[3].average, (*data)[3].scaling_factor);

  serial_bufWrite(toPrint2, -1);

}

// incorporate another scaling factor based on 1kW calibration values
void calc_1kW_scaling(gradient_data_struct *data){

  data->dbl_scale = (1000.0)/(double)(data->cts_at_1kW - data->offset);
}

void init_gradientData(gradient_data_struct* gradData, digital_filter* filter){
  int i = 0;
  gradData->filter = filter;
  // link up the elements of the list
  for(i = 0; i < NUM_TAPS - 1; i++){
    gradData->past_inputs[i].next = &(gradData->past_inputs[i+1]);
    gradData->past_inputs[i+1].prev = &(gradData->past_inputs[i]);
    gradData->past_outputs[i].next = &(gradData->past_outputs[i+1]);
    gradData->past_outputs[i+1].prev = &(gradData->past_outputs[i]);
  }
  // assert: i == NUM_TAPS - 1; link up the heads to the tails of both lists
  gradData->past_inputs[i].next = &(gradData->past_inputs[0]); // make circular
  gradData->past_inputs[0].prev = &(gradData->past_inputs[i]); 
  
  gradData->past_outputs[i].next = &(gradData->past_outputs[0]);// make circular
  gradData->past_outputs[0].prev = &(gradData->past_outputs[i]);

  // start with heads at beginnings of arrays
  gradData->inputs_head = gradData->past_inputs;
  gradData->outputs_head = gradData->past_outputs;
}

void set_target_pos(motor_struct* motor, long int pos){
  long int new_pos = pos;

  if(use_mm_pos){
    new_pos = mm_to_inch(pos);
  }

  if(abs(new_pos) <= (long int)max_travel){
    motor_enable(motor, 1); // enable the motor
    motor->target_pos = new_pos;
  }

    motor->step_bin = abs(motor->target_pos - motor->native_pos);

    if (motor->target_pos > motor->native_pos)
      motor->direction = 1;
    else
      motor->direction = 0;
}


long int inch_to_mm(long int inches){
  return (inches*inch_to_mm_scale) >> 13;
}

long int mm_to_inch(long int mm_x_100){
  return (mm_x_100*mm_to_inch_scale) >> 13;
}

// check limits of motor targets and step towards targets
void run_steppers(int up_or_down) {
  motor_struct* motor_pointer = &motorX;
  int i;
  for (i = 0; i < 2; i++) { // run once for each motor
    if(up_or_down == 1){
    // give a pulse to the stepper driver if it's time
    if (motor_pointer->target_pos == motor_pointer->native_pos
            || abs(motor_pointer->target_pos) > max_travel){
      if (motor_pointer->enabled)
        motor_enable(motor_pointer, 0); // disable motor
    }
    else {
      if(!motor_pointer->enabled)
        motor_enable(motor_pointer, 1); // enable motor
      // check if we need a positive step
      if (motor_pointer->target_pos > motor_pointer->native_pos) {
        step_on(motor_pointer, 1);
        motor_pointer->native_pos++;
      }  // otherwise check if we need a negative step
      else if (motor_pointer->target_pos < motor_pointer->native_pos) {
        step_on(motor_pointer, 0);
        motor_pointer->native_pos--;
      }
      motor_pointer->alt_pos = inch_to_mm(motor_pointer->native_pos);
     // motor_enable(motor_pointer, 0); // disable motor
    }
    }
    else{
      step_off(motor_pointer);
    }
    motor_pointer = &motorY; // now do the same as above for motorY
  }
}

/*toggle pause state and do one-time tasks to enter paused state*/
void pause_toggle(int* pause_flag){
  static long unsigned int time = 0;
  *pause_flag = !(*pause_flag);
  if(*pause_flag){ // system is paused
    serial_bufWrite("<m>System pausing</m>\n", -1);
    time = run_time;
  }
  else{ // system is unpaused
    serial_bufWrite("<m>System resuming</m>\n", -1);
    run_time = time;
  }
//  motorX.pwm.enable = !(*pause_flag);  // toggle enable pwm1 (off when paused)
//  motorY.pwm.enable = !(*pause_flag);
  testing = 0;
}

/* check input buffer for new serial input*/
void pollForSerial(command_struct* command) {
  static char commBuffer[BUFFER_SIZE];
  if (serial_gotNewline()){
    int inputLength = serial_available();
    serial_read(commBuffer, inputLength);
    parseInputString(command, commBuffer);
  }
}

/*takes a string and parses its three arguments */
void parseInputString(command_struct* command, char * string){
  // put three arguments into command structure
  sscanf(string, "%s %s %s", command->arg0, command->arg1, command->arg2);
  doCommand(command);
}

// sets up data structure for data posting
void initPostData(post_data* data, char* headings[], char* units[], int numCols){
  data->printHeader = 1;
  data->numColumns = numCols;
  data->columnLabels = headings;
  data->unitsLabels = units;
}

// write column headings to serial port
void postHeadings(post_data* data){
  serial_bufWrite("<h>", 3);
  int i;
  // print each column heading
  for (i = 0; i < data->numColumns; i++){
    serial_bufWrite(data->columnLabels[i], strlen(data->columnLabels[i]));
    if(i < data->numColumns - 1)
      serial_bufWrite("\t", 1);
  }
  serial_bufWrite("</h>\n", 5);
  postUnits(data);
  printHeader = 0; // reset flag
}

// send column units string over serial (wrapped in <u></u> tags)
void postUnits(post_data* data){
  serial_bufWrite("<u>", 3);
  int i;
  // print each column heading
  for (i = 0; i < data->numColumns; i++){
    serial_bufWrite(data->unitsLabels[i], strlen(data->unitsLabels[i]));
    if(i < data->numColumns - 1)
      serial_bufWrite("\t", 1);
  }
  serial_bufWrite("</u>\n", 5);
}

// print a row of data to serial port
void postRowData(post_data* data){
  // check if header should be printed
  if (printHeader)
    postHeadings(data);
  else{
    serial_bufWrite("<r>", 3);
    int i;
    // print each column heading
    for (i = 0; i < data->numColumns; i++){
      serial_bufWrite(data->dataRow[i], -1);
      if(i < data->numColumns - 1)
        serial_bufWrite("\t", 1);
    }
    serial_bufWrite("</r>\n", 5);
  }
}

// serial commands are all listed here

void doCommand(command_struct* command) {
  // command: p   -toggle pause state
  if (!strncmp(command->arg0, "pause", 5)) { // see if arg0 is "pause"
    pause_toggle(&pause_all);
  }    //  else if(!strncmp(command->arg0, "print", 5)){
    //    if(!strncmp(command->arg1, "header", 6)){
    //      printHeader = 1;
    //    }
    //    else cmdUnknown(command);
    //  }
  else if (!strncmp(command->arg0, "go", 2)) {
    sys_state = SYS_GO;
    serial_bufWrite("<m>System go!</m>\n", -2);
  } else if (!strncmp(command->arg0, "reset", 5)) {
    sys_state = SYS_RESET;
    serial_bufWrite("<m>Entering reset mode.</m>\n", -1);
  }    // test mode allows the system to accept command line setpoints
  else if (!strncmp(command->arg0, "calib", 5)) {
    if(atoi(command->arg2) == 0)
      calc_offsets(&quadrant);
  }
  else if (!strncmp(command->arg0, "set", 3)) {

    if (!strncmp(command->arg1, "sampletime", 10)) {
      // set sample period in milliseconds
      sample_time = atoi(command->arg2);
    } else if (!strncmp(command->arg1, "postrate", 8)) {
      // change data posting period in milliseconds
      post_period = atoi(command->arg2);
      serial_bufWrite("<m>Posting period set to ", -1);
      serial_bufWrite(command->arg2, -1);
      serial_bufWrite("</m>\n", -1);
    } else if (!strncmp(command->arg1, "blinkrate", 9)) {
      // change data posting period in milliseconds
      blink_period = atoi(command->arg2);
      serial_bufWrite("<m>Blink period set to ", -1);
      serial_bufWrite(command->arg2, -1);
      serial_bufWrite("</m>\n", -1);
    } else if (!strncmp(command->arg1, "steprate", 9)) {
      step_period = atoi(command->arg2);
      serial_bufWrite("<m>Step period set to ", -1);
      serial_bufWrite(command->arg2, -1);
      serial_bufWrite("</m>\n", -1);

    } else if (!strncmp(command->arg1, "x", 1)) {
      if(use_mm_pos){
        set_target_pos(&motorX, (long int) (atof(command->arg2)*100.0));
        //motorX.target_pos = mm_to_inch((long int)(atof(command->arg2)*100));
        sprintf(toPrint2, "<m>step bin: %d</m>\n", motorX.step_bin);
        serial_bufWrite(toPrint2, -1);
      }
      else
        set_target_pos(&motorX, (long int) (atoi(command->arg2)));
        //motorX.target_pos = atoi(command->arg2);
    } else if (!strncmp(command->arg1, "y", 1)) {
      if (use_mm_pos)
        set_target_pos(&motorY, (long int) (atof(command->arg2)*100.0));
        //motorY.target_pos = mm_to_inch((long int)(atof(command->arg2)*100));
      else
        set_target_pos(&motorY, (long int) (atoi(command->arg2)));
        //motorY.target_pos = atoi(command->arg2);
    } else if(!strncmp(command->arg1, "temp", 1)) {
      quadrant[TEMP_CHANNEL].offset = get_average(&(quadrant[TEMP_CHANNEL])) 
              - (long int)(atof(command->arg2)*10.0
              /quadrant[TEMP_CHANNEL].dbl_scale);
      sprintf(toPrint2, "<m>temp channel offset: %ld</m>\n", quadrant[TEMP_CHANNEL].offset);
      serial_bufWrite(toPrint2, -1);
      }
     else if (!strncmp(command->arg1, "alpha", 1)) {
      int i;
      for (i = 0; i < 5; i++) {
        quadrant[i].k1 = atoi(command->arg2);
        quadrant[i].k2 = 1000 - quadrant[i].k1;
      }
    }
    else cmdUnknown(command);
  } else if (!strncmp(command->arg0, "stop", 4)) {
    if (!strncmp(command->arg1, "posting", 7)) {
      postflag = 0;
      serial_bufWrite("<m>Posting stopped. Type 'start posting' to resume.</m>\n", -1);
    } else if (!strncmp(command->arg1, "motors", 4)) {
      motor_enable(&motorX, 0);
      motor_enable(&motorY, 0);
    } else cmdUnknown(command);

  } else if (!strncmp(command->arg0, "start", 5)) {
    if (!strncmp(command->arg1, "posting", 7)) {
      postflag = 1;
    }
  } else if (!strncmp(command->arg0, "xp", 2)) { // step commands
      if(use_mm_pos)
        set_target_pos(&motorX, inch_to_mm(motorX.target_pos) + (long int)(atof(command->arg2)*100.0));
//        motorX.target_pos += mm_to_inch((long int)(atof(command->arg1)*10));
      else
        set_target_pos(&motorX, motorX.target_pos +(long int) (atoi(command->arg2)));
//        motorX.target_pos += atoi(command->arg1);

  } else if (!strncmp(command->arg0, "xm", 2)) { // step commands

      if(use_mm_pos)
        set_target_pos(&motorX, inch_to_mm(motorX.target_pos) - (long int)(atof(command->arg2)*100.0));
//        motorX.target_pos -= mm_to_inch((long int)(atof(command->arg1)*10));
      else
        set_target_pos(&motorX, motorX.target_pos - (long int) (atoi(command->arg2)));
//        motorX.target_pos -= atoi(command->arg1);

  } else if (!strncmp(command->arg0, "yp", 2)) { // step commands
      if(use_mm_pos)
        set_target_pos(&motorY, inch_to_mm(motorY.target_pos) + (long int)(atof(command->arg2)*100.0));
        //motorY.target_pos = mm_to_inch((long int)(atof(command->arg2)*100));
      else
        set_target_pos(&motorY, motorY.target_pos +(long int) (atoi(command->arg2)));
        //motorX.target_pos = atoi(command->arg2);

  } else if (!strncmp(command->arg0, "ym", 2)) { // step commands
      if(use_mm_pos)
        set_target_pos(&motorY, inch_to_mm(motorY.target_pos) - (long int)(atof(command->arg2)*100.0));
//        motorY.target_pos -= mm_to_inch((long int)(atof(command->arg1)*10));
      else
        set_target_pos(&motorY, motorY.target_pos - (long int) (atoi(command->arg2)));
//        motorY.target_pos -= atoi(command->arg1);
  } else if (!strncmp(command->arg0, "deriv", 2)) { // get derivatives of signals
  sprintf(toPrint2, "<m>derivatives: time = %ld\na: %ld\nb: %ld\nc: %ld\nd: %ld</m>\n",
          run_time,
          differentiate(&(quadrant[0])),
          differentiate(&(quadrant[1])),
          differentiate(&(quadrant[2])),
          differentiate(&(quadrant[3])));

  serial_bufWrite(toPrint2, -1);

  }
  else if(!strncmp(command->arg0, "zero", 4)){
    if (!strncmp(command->arg1, "pos", 3)){
      sprintf(toPrint2,"<m>Zeroing carriage position from (x,y) = (%ld.%04d, %ld.%04d)</m>\n",
              *(motorX.display_pos)/10000L, abs(*(motorX.display_pos)%10000L),
              *(motorY.display_pos)/10000L, abs(*(motorY.display_pos)%10000L));
      motorX.native_pos = 0;
      motorY.native_pos = 0;
      motorX.alt_pos = 0;
      motorY.alt_pos = 0;
      motorX.target_pos = 0;
      motorY.target_pos = 0;

      serial_bufWrite(toPrint2, -1);
    }
  }
  else {
    cmdUnknown(command);
  }

}

// print error message to serial
void cmdUnknown(command_struct* command){
      serial_bufWrite("<m>Command unknown: ", -1);
      serial_bufWrite(command->arg0, -1);
      serial_bufWrite(" ", 1);
      serial_bufWrite(command->arg1, -1);
      serial_bufWrite(" ", 1);
      serial_bufWrite(command->arg2, -1);
      serial_bufWrite("</m>\n", 5);
}
