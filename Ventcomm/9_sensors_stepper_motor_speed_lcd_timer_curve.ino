//**************************
#include "Arduino.h"
#include "String.h"
#include "Variables.h"

//storage variables


void setup() {

  Serial.begin(9600);     // The Serial port of Arduino baud rate.
  Serial.println(F("Signum Techniks"));           // say hello to check serial line

  Serial2.begin(9600);

  //  delay(10);
  //  Serial2.print("$VSP20001&");
  //  delay(10);
  //  Serial2.print("$VSP30002&");
  //  delay(10);
  //  Serial2.print("$VSP40003&");
  //  delay(10);
  //  Serial2.print("$VSP50004&");
  //  delay(10);



  //coveyor motor step and direction
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  digitalWrite(MOTOR_STEP_PIN, HIGH);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW);

  //controls
  pinMode(MOTOR_RUN_PIN, INPUT_PULLUP);
  pinMode(HOME_SENSOR_PIN, INPUT_PULLUP);
  digitalWrite(MOTOR_RUN_PIN, HIGH);
  digitalWrite(HOME_SENSOR_PIN, HIGH);

  //Valves
  pinMode(EXHALE_VLV_PIN, OUTPUT);
  pinMode(INHALE_VLV_PIN, OUTPUT);
  pinMode(O2Cyl_VLV_PIN, OUTPUT);
  pinMode(O2Hln_VLV_PIN, OUTPUT);
  pinMode(INHALE_RELEASE_VLV_PIN, OUTPUT);

  digitalWrite(EXHALE_VLV_PIN, LOW);
  digitalWrite(INHALE_VLV_PIN, LOW);
  digitalWrite(O2Cyl_VLV_PIN, LOW);
  digitalWrite(O2Hln_VLV_PIN, LOW);
  digitalWrite(INHALE_RELEASE_VLV_PIN, LOW);

  calculate_value(home_speed_value * 10.0);

  //home cycle on power up
  home_cycle = true;
  motion_profile_count_temp = 0;
  //Serial.print("Home Cycle : ");
  run_pulse_count_temp = 0.0;
  run_pulse_count = 200000.0;
  digitalWrite(MOTOR_DIR_PIN, EXP_DIR);
  live_calculate_value(home_speed_value * 10.0);
  run_motor = true;

  delay(1000);


  //  //y = 3926850 + (2.261358 - 3926850)/(1 + (x/7122523000)^0.6693673)
  //
  //  float base = (795.0 / 7122523000);
  //  Stroke_length_new = 3926850.0 + (2.261358 - 3926850.0) / (1 + exp(base, 0.6693673));
  //  Serial.print("795 : "); Serial.println(Stroke_length_new);
  //
  //  base = (387.0 / 7122523000);
  //  Stroke_length_new = 3926850.0 + (2.261358 - 3926850.0) / (1 + pow(base, 0.6693673));
  //  Serial.print("387 : "); Serial.println(Stroke_length_new);
  //
  //  base = (96.0 / 7122523000);
  //  Stroke_length_new = 3926850.0 + (2.261358 - 3926850.0) / (1 + pow(base, 0.6693673));
  //  Serial.print("96 : "); Serial.println(Stroke_length_new);

  //Serial2.print("$VSP10000&");
  Read_ControlParams(READ_ALL_CTRL_PARAMS);

  delay(2000);
  calculate_speed_position();
  geCtrlState=CTRL_IDLE;
  Send_SystemState(geCtrlState);
}




void loop() {

  //Check if there is any packet received 
  if(UART2_IsCtrlPacketRecevied())
  {
    Ctrl_ProcessRxData();
  }
  // Control state machine periodic function 
  Ctrl_StateMachine_Manager();
}


void Ctrl_StateMachine_Manager(void)
{
  bool stateChanged = false;
  switch (geCtrlState)
  {
    case CTRL_IDLE:
    {
      if(cycle_start == true)
      {
        comp_start = true;
        comp_end = false;
        exp_start = false;
        exp_end = false;
        run_motor = true;
        geCtrlState=CTRL_COMPRESSION;
        Send_SystemState(geCtrlState);
      }
    }
    break;
    case CTRL_COMPRESSION:
    {      
      //compression started & is in progress
      if ((cycle_start == true) && (comp_start == true) && (comp_end == true)) 
      {
        //Serial2.print("$VSP10000&"); ask all parameter
        //Serial2.print("$VSSY0003&");   //expansion flag
        geCtrlState = CTRL_COMPRESSION_HOLD;
        Send_SystemState(geCtrlState);
        StateStartMillis = millis();
      }
    }
    break;
    case CTRL_COMPRESSION_HOLD:
    {
      if((millis() - StateStartMillis) > COMPRESSION_HOLD_TIME)
      {        
        cycle_start = true;
        comp_start = false;
        comp_end = false;
        exp_start = true;
        exp_end = false;
        run_motor = true;
        StateStartMillis = millis();
        geCtrlState = CTRL_EXPANSION;
        Send_SystemState(geCtrlState);
      }
    }
    break;
    case CTRL_EXPANSION:
    {
      //Expansion is in progress, wait till exp_end become true
      if ((cycle_start == true) && (exp_start == true) && (exp_end == true)) 
      {
        geCtrlState=CTRL_EXPANSION_HOLD;
        Send_SystemState(geCtrlState);
        StateStartMillis = millis();
      }
      else
      {
        //Expansion is in progress, wait
      }
    }
    break;
    case CTRL_EXPANSION_HOLD:
    {      
      if((millis() - StateStartMillis) > EXPANSION_HOLD_TIME)
      {
        if ((BPM_new != BPM) || (tidal_volume_new != tidal_volume) || (IER_new != IER)) 
        {
          BPM = BPM_new;
          Stroke_length = Stroke_length_new;
          inhale_ratio = 1.0;
          exhale_ratio = IER_new;
          calculate_speed_position();
        }
        cycle_start = true;
        comp_start = true;
        comp_end = false;
        exp_start = false;
        exp_end = false;
        run_motor = true;
        geCtrlState=CTRL_COMPRESSION;
        Send_SystemState(geCtrlState);
      }
      else
      {
        // Wait till end of the expansion of hold time
      }
    }
    break;
    case CTRL_INHALE_DETECTION:
    {
      
    }
    break;
    case CTRL_UNKNOWN_STATE:
    {

    }
    break;
    default:
    break;
  }
}


ISR(TIMER1_COMPA_vect) { //timer1 interrupt 1Hz toggles pin 13 (LED)
  //generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  if (run_motor == true) {
    if ((motion_profile_count_temp == 0) && (run_pulse_count_temp == 0.0)) {
      if ((exp_start == true) & (exp_end == false)) {
        //Serial.print("exp: "); Serial.println(motion_profile_count_temp);
        e_start_millis = millis();
        run_pulse_count = expansion_step_array[motion_profile_count_temp];
        digitalWrite(MOTOR_DIR_PIN, EXP_DIR);
        OCR1A = OCR1A_exp_array[motion_profile_count_temp] ;
        load_TCCR1B_var(TCCR1B_exp_array[motion_profile_count_temp]) ;
        Emergency_motor_stop = false;
        INHALE_RELEASE_VLV_CLOSE();
        INHALE_VLV_CLOSE();
        EXHALE_VLV_OPEN();
      }
      if ((comp_start == true) & (comp_end == false)) {
        //Serial.print("comp: "); Serial.println(motion_profile_count_temp);
        c_start_millis = millis();
        run_pulse_count = compression_step_array[motion_profile_count_temp];
        digitalWrite(MOTOR_DIR_PIN, COMP_DIR);
        OCR1A = OCR1A_comp_array[motion_profile_count_temp] ;
        load_TCCR1B_var(TCCR1B_comp_array[motion_profile_count_temp]);
        Emergency_motor_stop = false;
        INHALE_RELEASE_VLV_CLOSE();
        EXHALE_VLV_CLOSE();
        INHALE_VLV_OPEN();
      }
    }

    if (run_pulse_count_temp < run_pulse_count) {
      if (Emergency_motor_stop == false) digitalWrite(MOTOR_STEP_PIN, digitalRead(MOTOR_STEP_PIN) ^ 1);
      run_pulse_count_temp = run_pulse_count_temp + 0.5;
      if (home_cycle == true) {
        if (digitalRead(HOME_SENSOR_PIN) == 0) {
          run_motor = false;
          run_pulse_count_temp = 0.0;
          home_cycle = false;
          motion_profile_count_temp = 0;
          Serial.println("Home Cycle Complete...");
          if (cycle_start == true) int_start();
        }
      }
      if ((cycle_start == true) && (digitalRead(MOTOR_DIR_PIN) == EXP_DIR)) {
        if (digitalRead(HOME_SENSOR_PIN) == 0) {
          run_pulse_count_temp = run_pulse_count;
          motion_profile_count_temp = CURVE_EXP_STEPS;
          //motion_profile_count_temp = 0;
          //run_pulse_count_temp = 0.0;
        }
      }
    } else {
      //noInterrupts();
      run_motor = false;
      run_pulse_count_temp = 0.0;
      motion_profile_count_temp = motion_profile_count_temp + 1;
      if ((exp_start == true) & (exp_end == false)) {
        if (motion_profile_count_temp < CURVE_EXP_STEPS) {
          //Serial.print("exp: "); Serial.println(motion_profile_count_temp);
          run_pulse_count = expansion_step_array[motion_profile_count_temp];
          digitalWrite(MOTOR_DIR_PIN, EXP_DIR);
          OCR1A = OCR1A_exp_array[motion_profile_count_temp] ;
          load_TCCR1B_var(TCCR1B_exp_array[motion_profile_count_temp]) ;
          run_motor = true;
        } else {
          e_end_millis = millis();
          motion_profile_count_temp = 0;
          run_pulse_count_temp = 0.0;
          Emergency_motor_stop = false;
          INHALE_RELEASE_VLV_CLOSE();
          EXHALE_VLV_CLOSE();
          INHALE_VLV_OPEN();
          exp_end = true;
        }
      }
      if ((comp_start == true) & (comp_end == false)) {
        if (motion_profile_count_temp < CURVE_COMP_STEPS) {
          //Serial.print("comp: "); Serial.println(motion_profile_count_temp);
          run_pulse_count = compression_step_array[motion_profile_count_temp];
          digitalWrite(MOTOR_DIR_PIN, COMP_DIR);
          OCR1A = OCR1A_comp_array[motion_profile_count_temp] ;
          load_TCCR1B_var(TCCR1B_comp_array[motion_profile_count_temp]) ;
          run_motor = true;
        } else {
          c_end_millis = millis();
          motion_profile_count_temp = 0;
          run_pulse_count_temp = 0.0;
          Emergency_motor_stop = false;
          INHALE_RELEASE_VLV_CLOSE();
          INHALE_VLV_CLOSE();
          EXHALE_VLV_OPEN();
          comp_end = true;
        }
      }
      //interrupts();
    }
  }
}



void load_TCCR1B_var(int TCCR1B_var_temp) {
  if (TCCR1B_var_temp == 1)    {
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  }
  if (TCCR1B_var_temp == 8)    {
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  }
  if (TCCR1B_var_temp == 64)   {
    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  }
  if (TCCR1B_var_temp == 256)  {
    TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
  }
  if (TCCR1B_var_temp == 1024) {
    TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
  }

  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
}


void calculate_speed_position() {
  //Serial.println(("Speed curve calculations : "));

  //BPM = 7.0;
  //tidal_volume = 600.0;
  inhale_ratio = 1.0;
  //exhale_ratio = 2.0;
  //Stroke_length = 80.00;

  //  if (tidal_volume == 50.0) Stroke_length = 16.0;
  //  if (tidal_volume == 100.0) Stroke_length = 24.0;
  //  if (tidal_volume == 150.0) Stroke_length = 31.0;
  //  if (tidal_volume == 200.0) Stroke_length = 37.0;
  //  if (tidal_volume == 250.0) Stroke_length = 42.0;
  //  if (tidal_volume == 300.0) Stroke_length = 48.0;
  //  if (tidal_volume == 350.0) Stroke_length = 53.0;
  //  if (tidal_volume == 400.0) Stroke_length = 57.0;
  //  if (tidal_volume == 450.0) Stroke_length = 62.0;
  //  if (tidal_volume == 500.0) Stroke_length = 66.0;
  //  if (tidal_volume == 550.0) Stroke_length = 70.0;
  //  if (tidal_volume == 600.0) Stroke_length = 74.0;
  //  if (tidal_volume == 650.0) Stroke_length = 78.0;
  //  if (tidal_volume == 700.0) Stroke_length = 82.0;
  //  if (tidal_volume == 750.0) Stroke_length = 86.0;
  //  if (tidal_volume == 800.0) Stroke_length = 90.0;

  float cycle_time = 60.0 / BPM;
  float inhale_time = (cycle_time * inhale_ratio) / (inhale_ratio + exhale_ratio);
  float exhale_time = (cycle_time * exhale_ratio) / (inhale_ratio + exhale_ratio);
  float inhale_vpeak = ((Stroke_length * 0.8) / (inhale_time * 0.8)) ;
  float exhale_vpeak = ((Stroke_length * 0.8) / (exhale_time * 0.8));

  compression_speed = ( inhale_vpeak / LEAD_SCREW_PITCH ) * 60;
  expansion_speed =   ( exhale_vpeak / LEAD_SCREW_PITCH ) * 60;

  //this 1.8 degree step motor so 200 steps for 360 degree
  run_pulse_count_1_full_movement = ((micro_stepping * (Stroke_length / LEAD_SCREW_PITCH * 1.0)) / 2.0);
  run_pulse_count_1_piece_compression = (run_pulse_count_1_full_movement / 100.0);   //CURVE_COMP_STEPS   '''taking 100 pieces to ease the % calculation
  run_pulse_count_1_piece_expansion = (run_pulse_count_1_full_movement / 100.0);     //CURVE_EXP_STEPS  '''taking 100 pieces to ease the % calculation

  //avoid accel/deccel below 300 rpm
  if (compression_speed > MIN_RPM_NO_ACCEL) {
    compression_speed = compression_speed + (compression_speed * 0.08);
    compression_min_speed = MIN_RPM_NO_ACCEL;
  } else compression_min_speed = compression_speed;
  if (expansion_speed > MIN_RPM_NO_ACCEL) {
    expansion_speed = expansion_speed + (expansion_speed * 0.08);
    expansion_min_speed = MIN_RPM_NO_ACCEL;
  } else expansion_min_speed = expansion_speed;

  compression_step_array[10] = run_pulse_count_1_piece_compression * 80;
  compression_speed_array[10] = compression_speed;
  pre_calculate_value(compression_speed_array[10] * 10.0);
  TCCR1B_comp_array[10] = TCCR1B_var;
  OCR1A_comp_array[10] = OCR1A_var;

  expansion_step_array[10] = run_pulse_count_1_piece_expansion * 80;
  expansion_speed_array[10] = expansion_speed;
  pre_calculate_value(expansion_speed_array[10] * 10.0);
  TCCR1B_exp_array[10] = TCCR1B_var;
  OCR1A_exp_array[10] = OCR1A_var;

  float comp_slope = (compression_speed - compression_min_speed) / 8.0;
  float exp_slope = (expansion_speed - expansion_min_speed) / 8.0;
  int i;
  for (i = 0; i < 10; i++) {
    compression_step_array[i] = run_pulse_count_1_piece_compression;
    compression_step_array[20 - i] = run_pulse_count_1_piece_compression;
    compression_speed_array[i] = (comp_slope * ((Stroke_length * 0.01) * (i ) )) + compression_min_speed;
    compression_speed_array[20 - i] = (comp_slope * ((Stroke_length * 0.01) * (i ) )) + compression_min_speed;

    //compression_speed_array[i] = compression_min_speed + (i * ((compression_speed - compression_min_speed) / 10.0));
    //compression_speed_array[20 - i] = compression_min_speed + (i * ((compression_speed - compression_min_speed) / 10.0));

    pre_calculate_value(compression_speed_array[i] * 10.0);
    TCCR1B_comp_array[i] = TCCR1B_var;
    OCR1A_comp_array[i] = OCR1A_var;
    TCCR1B_comp_array[20 - i] = TCCR1B_var;
    OCR1A_comp_array[20 - i] = OCR1A_var;

    expansion_step_array[i] = run_pulse_count_1_piece_expansion;
    expansion_step_array[20 - i] = run_pulse_count_1_piece_expansion;
    expansion_speed_array[i] = (exp_slope * ((Stroke_length * 0.01) * (i ))) + expansion_min_speed;
    expansion_speed_array[20 - i] = (exp_slope * ((Stroke_length * 0.01) * (i ))) + expansion_min_speed;

    //expansion_speed_array[i] = expansion_min_speed + (i * ((expansion_speed - expansion_min_speed) / 10.0));
    //expansion_speed_array[20 - i] = expansion_min_speed + (i * ((expansion_speed - expansion_min_speed) / 10.0));

    pre_calculate_value(expansion_speed_array[i] * 10.0);
    TCCR1B_exp_array[i] = TCCR1B_var;
    OCR1A_exp_array[i] = OCR1A_var;
    TCCR1B_exp_array[20 - i] = TCCR1B_var;
    OCR1A_exp_array[20 - i] = OCR1A_var;
  }

  //  for (i = 0; i < 21; i++) {
  //    Serial.print("Compression: "); Serial.print(i); Serial.print(" | step: "); Serial.print(compression_step_array[i]); Serial.print(" | rpm: "); Serial.println(compression_speed_array[i]);
  //    Serial.print("expansion  : "); Serial.print(i); Serial.print(" | step: "); Serial.print(expansion_step_array[i]);  Serial.print(" | rpm: "); Serial.println(expansion_speed_array[i]);
  //  }
}



void stop_timer() {
  //cli();  // One way to disable the timer, and all interrupts

  TCCR1B &= ~(1 << CS12); // turn off the clock altogether
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);

  TIMSK1 &= ~(1 << OCIE1A); // turn off the timer interrupt
}



void pre_calculate_value(float rpm) {

  double freq;
  //  long OCR1A_var;
  //  long TCCR1B_var;

  rpm = abs(rpm);
  freq = round( long((rpm * micro_stepping) / 600.0));
  //Serial.print(("rpm  : ")); Serial.print(rpm / 10); Serial.print(("  Freq : ")); Serial.print(freq); Serial.print(("  Micro stepping: ")); Serial.println(micro_stepping);

  // initialize timer1
  //noInterrupts(); // disable all interrupts
  //set timer1 interrupt at 1Hz
  //TCCR1A = 0;// set entire TCCR1A register to 0
  //TCCR1B = 0;// same for TCCR1B
  //TCNT1  = 0;//initialize counter value to 0

  // set compare match register for 1hz increments
  TCCR1B_var = 1;
  OCR1A_var = (16000000.0 / (freq * TCCR1B_var)) - 1; // (must be <65536)
  //  Serial.println("1 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  if (OCR1A_var > 65536 || OCR1A_var <= 0 ) {
    TCCR1B_var = 8;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("2 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 64;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1; // (must be <65536)
  }
  //  Serial.println("3 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 256;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("4 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 1024;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("5 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

}

void live_calculate_value(float rpm) {
  double freq;
  //  long OCR1A_var;
  //  long TCCR1B_var;

  //  if (rpm >= 0.0) {
  //    digitalWrite(MOTOR_DIR_PIN, HIGH);
  //    Serial.print(("Dir  : Clockwise"));
  //  } else {
  //    digitalWrite(MOTOR_DIR_PIN, LOW);
  //    Serial.print(("Dir  : Non-clockwise"));
  //  }

  rpm = abs(rpm);
  freq = round( long((rpm * micro_stepping) / 600.0));
  //Serial.print(("rpm  : ")); Serial.print(rpm / 10); Serial.print(("  Freq : ")); Serial.print(freq); Serial.print(("  Micro stepping: ")); Serial.println(micro_stepping);

  // initialize timer1
  //noInterrupts(); // disable all interrupts
  //set timer1 interrupt at 1Hz
  //TCCR1A = 0;// set entire TCCR1A register to 0
  //TCCR1B = 0;// same for TCCR1B
  //TCNT1  = 0;//initialize counter value to 0

  // set compare match register for 1hz increments
  TCCR1B_var = 1;
  OCR1A_var = (16000000.0 / (freq * TCCR1B_var)) - 1; // (must be <65536)
  //  Serial.println("1 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  if (OCR1A_var > 65536 || OCR1A_var <= 0 ) {
    TCCR1B_var = 8;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("2 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 64;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1; // (must be <65536)
  }
  //  Serial.println("3 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 256;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("4 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 1024;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("5 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  OCR1A = OCR1A_var;

  // Set CS12, CS11 and CS10 bits for X prescaler
  //001 = 1
  //010 = 8
  //011 = 64
  //100 = 256
  //101 = 1024
  if (TCCR1B_var == 1)    {
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  }
  if (TCCR1B_var == 8)    {
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  }
  if (TCCR1B_var == 64)   {
    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  }
  if (TCCR1B_var == 256)  {
    TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
  }
  if (TCCR1B_var == 1024) {
    TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
  }

  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // enable timer compare interrupt
  //TIMSK1 |= (1 << OCIE1A);
  //interrupts(); // enable all interrupts
}

void calculate_value(float rpm) {
  double freq;
  //  long OCR1A_var;
  //  long TCCR1B_var;


  //  if (rpm >= 0.0) {
  //    digitalWrite(MOTOR_DIR_PIN, HIGH);
  //    Serial.print(("Dir  : Clockwise"));
  //  } else {
  //    digitalWrite(MOTOR_DIR_PIN, LOW);
  //    Serial.print(("Dir  : Non-clockwise"));
  //  }


  rpm = abs(rpm);
  //  Serial.print(("rpm  : ")); Serial.println(rpm / 10);
  //  Serial.print(("Micro: ")); Serial.println(micro_stepping);
  freq = round( long((rpm * micro_stepping) / 600.0));
  //  Serial.print(("Freq : ")); Serial.println(freq);


  // initialize timer1
  noInterrupts(); // disable all interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0

  // set compare match register for 1hz increments
  TCCR1B_var = 1;
  OCR1A_var = (16000000.0 / (freq * TCCR1B_var)) - 1; // (must be <65536)
  //  Serial.println("1 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);
  if (OCR1A_var > 65536 || OCR1A_var <= 0 ) {
    TCCR1B_var = 8;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("2 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);
  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 64;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1; // (must be <65536)
  }
  //  Serial.println("3 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);
  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 256;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("4 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);
  if (OCR1A_var > 65536 || OCR1A_var <= 0) {
    TCCR1B_var = 1024;
    OCR1A_var = ((16000000) / (freq * TCCR1B_var)) - 1;  // (must be <65536)
  }
  //  Serial.println("5 :");
  //  Serial.print(("TCCR : ")); Serial.print(TCCR1B_var);
  //  Serial.print(("       OCR1 : ")); Serial.println(OCR1A_var);

  OCR1A = OCR1A_var;

  // Set CS12, CS11 and CS10 bits for X prescaler
  //001 = 1
  //010 = 8
  //011 = 64
  //100 = 256
  //101 = 1024
  if (TCCR1B_var == 1)    {
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  }
  if (TCCR1B_var == 8)    {
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  }
  if (TCCR1B_var == 64)   {
    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  }
  if (TCCR1B_var == 256)  {
    TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
  }
  if (TCCR1B_var == 1024) {
    TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
  }

  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  interrupts(); // enable all interrupts
}
