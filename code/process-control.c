/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file process-control.c
 *
 * @brief Variables and functions related to the state and controll of a process
 *
 * - Compiler:          gcc-avr
 * - Project:           LiFePO4 Charge Controller
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: July 2014
 *****************************************************************************/
#include "process-control.h"

/**
 * @brief Variables that control whether the process_control is running
 * 
 * @var begin_process_control_flag
 * Flag that indicates that process should run
 * @var process_control_running_flag
 * Flag that indicates process_control is running
 * @var process_control_enable
 * Flag for global enable/disable of process_control
 **/

uint8_t begin_process_control_flag = 0;
uint8_t process_control_running_flag = 0;
uint8_t processControlEnable = 0;               //disabled by default

// Variables used for indicator LED's
uint8_t indicatorPosition = 0;
uint8_t indicatorCounterRaw = 0;

/**
 * @brief Variables used by the actual programmes running in calculate_outputs
 * 
 * @var setpoint_reached_counter
 * When the setpoint is reached start counting cycles that this is true
 * @var rest_count
 * How long have we been resting for. May be redundant if we decriment rest timer.
 * @var rest_init_flag
 * Allow us to run some stuff on the first rest
 * @var done_init_flag
 * Same as above
 **/

uint8_t setpoint_reached_counter = 0;
uint8_t rest_init_flag = 0;
uint8_t done_init_flag = 0;
uint8_t startPSUflag = 0;
uint32_t Ah_QuarterSecond = 0;

/** @brief struct(s) for PID **/
struct u_PID_DATA pidData_cc;    // PID data for constant current
struct u_PID_DATA pidData_cv;    // PID data for constant voltage
// = {   // PID data for constant voltage

/** @brief struct Process process for program 1 **/
struct Process process = {
  {0},
  {
    .pwm_duty = PWM_START,
    .PSU_state = 20,
    .error_code = 0,
    .charge_mode = CHARGE_MODE_CONSTANT_CURRENT, // The initial charge mode
    .charge_retries = 0,
    .last_charge_mode = CHARGE_MODE_CONSTANT_CURRENT,
    .time_wait_retry = 0,
    .led_green = 0,
    .led_red = 0
  },
  {
    .program_number               = 1,
    .charge_retries_max           = SETTINGS_CHARGE_RETRIES_MAX,
    // CC
    .current_cc                   = SETTINGS_CURRENT_CC,
    .voltage_cc                   = SETTINGS_VOLTAGE_CC_CHARGED,
    .BMS_max_voltage_cc           = SETTINGS_BMS_MAX_VOLTAGE_CC,
    .pid_proportion_cc            = SETTINGS_PID_PROPORTION_CC,
    // CV
    .BMS_max_voltage_cv           = SETTINGS_BMS_MAX_VOLTAGE_CV,
    .voltage_cv                   = SETTINGS_VOLTAGE_CV,
    .current_cv                   = SETTINGS_CURRENT_CV,
    .current_cv_done              = SETTINGS_CURRENT_CV_DONE,
    .pid_proportion_cv            = SETTINGS_PID_PROPORTION_CV,
    .pid_proportion_cv_bms        = SETTINGS_PID_PROPORTION_CV_BMS,
    // Balancing
    .BMS_max_voltage_balancing    = SETTINGS_BMS_MAX_VOLTAGE_BALANCING,
    .current_balancing            = SETTINGS_CURRENT_BALANCING,
    .voltage_balancing            = SETTINGS_VOLTAGE_BALANCING,
    .pid_proportion_balancing     = SETTINGS_PID_PROPORTION_BALANCING,
    .pid_proportion_balancing_bms = SETTINGS_PID_PROPORTION_BALANCING_BMS,
    // Limits
    .power_max                    = SETTINGS_POWER_MAX,
    .current_max                  = SETTINGS_CURRENT_MAX,
    .voltage_max                  = SETTINGS_VOLTAGE_MAX,
    .voltage_min                  = SETTINGS_VOLTAGE_MIN,
    .max_battery_temp             = SETTINGS_MAX_BATTERY_TEMP,
    /** PID **/
    .PIDoutput = 0,
    /** Voltage calibration **/
    .analog_voltage_offset_code   = ANALOG_VOLTAGE_HIGH_A_OFFSET_CODE,
    .analog_voltage_slope_code    = ANALOG_VOLTAGE_HIGH_A_SLOPE_CODE,
    /** Current calibration **/
    .analog_current_offset_code   = ANALOG_CURRENT_OFFSET_CODE,
    .analog_current_slope_code    = ANALOG_CURRENT_SLOPE_CODE
  }
};

/**
 * @brief Get the state of the system, input values and output states
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void get_state(struct Process *process) {
  struct Inputs *inputs = &process->inputs;
  struct Outputs *outputs = &process->outputs;
  struct Settings *settings = &process->settings;
  
  //inputs->current = read_ADC_pin(A_SENSE_ADC_PIN, A_SENSE_ADC_REF);
  //inputs->current = get_analog_current(ANALOG_CURRENT_SLOPE_CODE, ANALOG_CURRENT_OFFSET_CODE);
   inputs->current = get_analog_current(
     settings->analog_current_slope_code,
     settings->analog_current_offset_code
   );
    
  // Get the voltage
  //inputs->voltage = get_analog_current(ANALOG_CURRENT_SLOPE_CODE, ANALOG_CURRENT_OFFSET_CODE);
  //inputs->voltage = read_ADC_pin(A_SENSE_ADC_PIN, A_SENSE_ADC_REF);
  inputs->voltage = get_voltage(
    settings->analog_voltage_slope_code,
    settings->analog_voltage_offset_code
  );
  
  // Get BMS status
  bmscomms_find_and_process_packet(&inputs->BMS_max_voltage, &inputs->BMS_status, &inputs->BMS_balancing);
  
}

/**
 * @brief Enable and power up the PSU output
 * 
 * @param *process Process Struct
 **/
void psu_enable(struct Process* process)
{
  struct Outputs  *outputs  = &process->outputs;
  struct Settings *settings = &process->settings;
  
  // Set the duty cycle to safe start condition
  outputs->pwm_duty = PWM_START;
  // Start the PSU's
  psu_power(3,1);
  // Note that we have started the psu's
  startPSUflag = 1;
  // Wait a bit
  _delay_ms(500);
}
/**
 * @brief Disable and power down the PSU outputs
 * 
 * @param *process Process Struct
 **/
void psu_disable(struct Process* process)
{
  struct Outputs  *outputs  = &process->outputs;
  struct Settings *settings = &process->settings;
  
  // Set the duty cycle and PIDOutput to safe start condition
  outputs->pwm_duty = PWM_START;
  settings->PIDoutput = 0;
  // Stop the PSU's
  psu_power(3,0);
  // Note that we have stopped the PSU's
  startPSUflag = 0;
}


/**
 * @brief Find the current setpoint given a current, a voltage, and a maximum current
 * 
 * @param power The desired power in Watts
 * @param current_max The maximum current in A*1000
 * @param measured_voltage The measured voltage in V*10
 * @return The currrent required to meet CP conditions
 **/
uint16_t cp_current_calc(uint16_t power_set, uint16_t current_max, uint16_t measured_voltage)
{
  return min(
    // Power is in Watts (VA), whereas voltage is V*10, and current is in A*1000, must multiply by 10,000
    // IE V = 100x10 = 1,000, C = 20*1000 = 20,000, P = 20,000,000, P' = 2000 * 1000
    ((uint32_t)power_set*10000)/measured_voltage,
    current_max
  );
}

/**
 * @brief Calculate the outputs in order to controll the process
 *
 * The process-control is primarily calculating outputs.
 * This process is broken up as follows:
 *   - Check `outputs->charge_mode' and calculate outputs accordingly
 *   - Check the calculated outputs are reasonable, adjust if nesc.
 *   - Set the outputs to calculated values
 *   - Log every second and respond to serial commands
 * 
 *   Charging the batteries follows the follows the following stages
 *   1. Constant current charge (CHARGE_MODE_CONSTSANT_CURRENT)
 *      Charge at a constant current (settings->current_charge) until the
 *      input voltage (inputs->voltage) reaches the setpoint
 *      (settings->voltage_charged)
 *   2. Absorbtion charge (CHARGE_MODE_ABSORBTION)
 *      No charging for a multiple of the constant current charge time
 *   3. Constant voltage charge (CHARGE_MODE_CONSTANT_VOLTAGE)
 *      Charge at a constant voltage (settings->voltage_float) for a multiple
 *      of the constant current charge time, whilst ensuring the current does
 *      not exceed settings->current_float
 *
 * 
 * @param *process a struct of type Process in which to save the system state
 **/
void calculate_outputs(struct Process* process)
{
  struct Inputs   *inputs   = &process->inputs;
  struct Outputs  *outputs  = &process->outputs;
  struct Settings *settings = &process->settings;
  
  switch(outputs->charge_mode) {
    //Constant Current PID
    case CHARGE_MODE_CONSTANT_CURRENT:
      //Check if the voltage set point has been reached - then cv charge
      if ( inputs->voltage >= settings->voltage_cc ||
        inputs->BMS_max_voltage >= settings->BMS_max_voltage_cv ||
        inputs->BMS_balancing >= BMSCOMMS_BALANCE_SOME
      ) {
        
        // Check if the setpoint has been reached enough times
        setpoint_reached_counter++;
        if ( setpoint_reached_counter >= SETPOINT_REACHED_COUNT )
        {
          //Reset setpoint counter
          setpoint_reached_counter = 0;
          //Change to CHARGE_MODE_CONSTANT_VOLTAGE
          settings->analog_voltage_slope_code = ANALOG_VOLTAGE_LOW_A_SLOPE_CODE;
          settings->analog_voltage_offset_code = ANALOG_VOLTAGE_LOW_A_OFFSET_CODE;
          outputs->charge_mode = CHARGE_MODE_CONSTANT_VOLTAGE;
          break;
        }
      } else {
      // Setpoint is not reached.
        // Decriment the setpoint counter
        if ( setpoint_reached_counter > 0 ) 
        {
          setpoint_reached_counter--;
        }
      }
        
        //If the PSU's are not on/started they must be started/turned-on.
        if(!startPSUflag)
        {
          psu_enable(process);
        }
        
        //Constant current PID
        settings->PIDoutput = pid_proportional_simple(
          // Measured value
          inputs->current,
          // Set point
          settings->current_cc,
          // (P)roportion
          settings->pid_proportion_cc,
          // Maximum output value
          SETTINGS_PID_MAX_CC
        );
      break;
      
    // Constant Voltage PID
    // Charge at the voltage of the highest cell - just below Balance voltage
    // Stop when a cell is balancing
    case CHARGE_MODE_CONSTANT_VOLTAGE:
      // Check if constant voltage charge stage has been going long enough
      if ( inputs->current <= settings->current_cv_done ||
            inputs->voltage >= settings->voltage_cv ||
            inputs->BMS_balancing >= BMSCOMMS_BALANCE_SOME ) 
      {        
        // Check if the setpoint has been reached enough times
        setpoint_reached_counter++;
        
        if ( setpoint_reached_counter >= SETPOINT_REACHED_COUNT )
        {
          //Turn off the PSU's and set the next stage
          psu_disable(process);
          //Reset setpoint counter
          setpoint_reached_counter = 0;
          //Change to CHARGE_MODE_ABSORBTION
          outputs->charge_mode = CHARGE_MODE_BALANCE;
          break;
        }
      }
      else 
      {
        //If the PSU's are not on/started they must be started/turned-on.
        if(!startPSUflag)
        {
          //Start the PSU
          psu_enable(process);
        }
        
        // Check if the current is greater than the float current
        if( inputs->current >= settings->current_cv )
        {
          settings->PIDoutput = pid_proportional_simple(
            // Measured value
            inputs->current,
            // Set point
            settings->current_cv,
            // (P)roportion
            settings->pid_proportion_cc,
            // Maximum output value
            SETTINGS_PID_MAX_CC
          );
        } else {
          settings->PIDoutput = pid_proportional_simple(
            // Measured value
            inputs->BMS_max_voltage,
            // Set point
            settings->BMS_max_voltage_cv,
            // Proportional
            settings->pid_proportion_cv_bms,
            // Output Max
            SETTINGS_PID_MAX_CV_BMS
          );
        }
        break;
      }
      
    case CHARGE_MODE_BALANCE: //Constant Current PID
      //Check if cells have finished balancing or the voltage set point has been reached.
      // We detect not balancing as the balancing output turns off when all cells are done
      if ( inputs->BMS_balancing >= BMSCOMMS_BALANCE_ALL || 
           inputs->voltage >= settings->voltage_balancing) {
        
        // Check if the setpoint has been reached enough times
        setpoint_reached_counter++;
        if ( setpoint_reached_counter >= SETPOINT_REACHED_COUNT )
        {
          //Turn off the PSU's and set the next stage
          psu_disable(process);
          //Reset setpoint counter
          setpoint_reached_counter = 0;
          //Change to CHARGE_MODE_ABSORBTION
          outputs->charge_mode = CHARGE_MODE_OFF;
          break;
        }
      } else {
      // Setpoint is not reached.
        // Decriment the setpoint counter
        if ( setpoint_reached_counter > 0 ) 
        {
          setpoint_reached_counter--;
        }
      }
      //If the PSU's are not on/started they must be started/turned-on.
      if(!startPSUflag)
      {
        psu_enable(process);
      }
      
      // Check if the current is greater than the float current
      if( inputs->BMS_max_voltage >= settings->BMS_max_voltage_balancing )
      {
        settings->PIDoutput = pid_proportional_simple(
          // Measured value
          inputs->BMS_max_voltage,
          // Set point
          settings->BMS_max_voltage_balancing,
          // Proportional
          settings->pid_proportion_balancing_bms,
          // Output Max
          SETTINGS_PID_MAX_BALANCING_BMS
        );
      } else {
      //Constant current PID
      settings->PIDoutput = pid_proportional_simple(
        // Measured value
        inputs->current,
        // Set point
        settings->current_balancing,
        // (P)roportion
        settings->pid_proportion_balancing,
        // Maximum output value
        SETTINGS_PID_MAX_BALANCING
      );
      }
      break;
      
      // Special mode for balancing unil set total volage
      case CHARGE_MODE_SPECIAL: //Constant Current PID=
      if ( inputs->voltage >= settings->voltage_balancing) {
        
        // Check if the setpoint has been reached enough times
        setpoint_reached_counter++;
        if ( setpoint_reached_counter >= SETPOINT_REACHED_COUNT )
        {
          //Turn off the PSU's and set the next stage
          psu_disable(process);
          //Reset setpoint counter
          setpoint_reached_counter = 0;
          //Change to CHARGE_MODE_ABSORBTION
          outputs->charge_mode = CHARGE_MODE_OFF;
          break;
        }
      }  else {
      // Setpoint is not reached.
        // Decriment the setpoint counter
        if ( setpoint_reached_counter > 0 ) 
        {
          setpoint_reached_counter--;
        }
      }
      //If the PSU's are not on/started they must be started/turned-on.
      if(!startPSUflag)
      {
        psu_enable(process);
      }
      
      //Constant current PID
      settings->PIDoutput = pid_proportional_simple(
        // Measured value
        inputs->current,
        // Set point
        settings->current_balancing,
        // (P)roportion
        settings->pid_proportion_balancing,
        // Maximum output value
        SETTINGS_PID_MAX_BALANCING
      );
      break;
      
    case CHARGE_MODE_RETRY_WAIT:
    {
      // Check if we have waited long enough to try again
      if ( timestamp >= outputs->time_wait_retry )
      {
        outputs->charge_mode = outputs->last_charge_mode;
      }
    }
    default:
    {
      //Turn off the PSU's
      psu_disable(process);
    }
  }
  
  //Invert PWM output is inverted, so we decrement it with PIDoutput
  outputs->pwm_duty -= settings->PIDoutput;
  // Maintain PWM Limits/Boundaries
  if ( outputs->pwm_duty >= PWM_TOP-1 )
  {
    outputs->pwm_duty = PWM_TOP-2;
  }
  if ( outputs->pwm_duty <= 0 )
  {
    outputs->pwm_duty = 0;
  }
}

void handle_retries(struct Process* process)
{
  struct Inputs   *inputs   = &process->inputs;
  struct Outputs  *outputs  = &process->outputs;
  struct Settings *settings = &process->settings;
  
  // First lets turn it all off
  psu_disable(process);
  outputs->charge_mode = CHARGE_MODE_OFF_ERROR;
  
//   // Check number of retries
//   if ( outputs->charge_retries >= settings->charge_retries_max ) 
//   {
//     // We are done
//     send_string_p(PSTR("Maximum number of retries reached, aborting...\r\n"));
//     outputs->charge_mode = CHARGE_MODE_OFF_ERROR;
//   }
//   else
//   {
//     // Incriment retries
//     outputs->charge_retries++;
//     // Set the charge mode to CHARGE_MODE_RETRY_WAIT
//     outputs->charge_mode = CHARGE_MODE_RETRY_WAIT;
//     // Set the retry wait time
//     outputs->time_wait_retry = timestamp + SETTINGS_CHARGE_RETRY_WAIT_TIME;
//   }
}
  
/**
 * @brief Check that no inputs or proposed outputs will break anything
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void check_limits(struct Process* process)
{
  struct Inputs   *inputs   = &process->inputs;
  struct Outputs  *outputs  = &process->outputs;
  struct Settings *settings = &process->settings;
 
  // Initialise static bms_error_count
  static uint8_t bms_error_count = 0;
  if (bms_error_count > 0 ) bms_error_count--;

  uint8_t halt = 0;
  
  if (outputs->charge_mode != CHARGE_MODE_OFF_ERROR){
    //Check if any of the stop conditions are reached.
    if (inputs->current >= settings->current_max) {
      halt++;
      send_string_p(PSTR("Over current!"));
      send_newline();
    }
    if (inputs->voltage >= settings->voltage_max) {
      halt++;
      send_string_p(PSTR("Over voltage!"));
      send_newline();
    }
//     if (inputs->voltage <= settings->voltage_min) {
//       halt++;
//       send_string_p(PSTR("Under voltage!"));
//       send_newline();
//     }
    if (inputs->BMS_status != BMSCOMMS_STATUS_OK &&
        ++bms_error_count >= SETTINGS_BMSCOMMS_ERROR_COUNT) {
      if (inputs->BMS_status == BMSCOMMS_STATUS_FAULT) {
        halt++;
        send_string_p(PSTR("BMS comms fault!"));
        send_newline();
      }
      if (inputs->BMS_status == BMSCOMMS_STATUS_OVER_VOLTAGE) {
        halt++;
        send_string_p(PSTR("BMS battery over voltage!"));
        send_newline();
      }
      if (inputs->BMS_status == BMSCOMMS_STATUS_OVER_TEMP) {
        halt++;
        send_string_p(PSTR("BMS battery over temperature!"));
        send_newline();
      }
      if (inputs->BMS_status == BMSCOMMS_STATUS_UNDER_VOLTAGE) {
        halt++;
        send_string_p(PSTR("BMS battery under voltage!"));
        send_newline();
      }
      if (inputs->BMS_status > BMSCOMMS_STATUS_UNDER_VOLTAGE) {
        halt++;
        send_string_p(PSTR("BMS unknown fault!"));
        send_newline();
      }
    }
    if(halt != 0){
      outputs->last_charge_mode = outputs->charge_mode;
      send_string_p(PSTR("A stop condition was breached. Shutting down. and retrying\r\n"));
      handle_retries(process);
    }
  }
}

/**
 * @brief Calculate the led light output for progress indication.
 * 
 * The process control happens in a medium timesetep given by
 * MEDIUM_TIME_INTERVAL, which is 125/55 = MEDIUM_TIME_FRACTION
 * being 5 times per send. We will denote counts (1,2,3,4,5) as 1/5th of a
 * second, of colour (r/g) and _ for off
 * 
 * There are green and red LED's which will have the following patterns for
 * the following charge states
 * CHARGE_MODE_OFF
 *   Solid green
 *   gggggggggg
 * CHARGE_MODE_CONSTANT_CURRENT
 *   Rapid green blink then pause
 *   g_g_g_____
 * CHARGE_MODE_ABSORBTION
 *   Long 50% duty cycle green blink
 *   ggggg_____
 * CHARGE_MODE_CONSTANT_VOLTAGE
 *   Slower green blink then pause
 *   gg__gg____
 * CHARGE_MODE_CONSTANT_POWER
 *   Rapid green blink alternating with red then pause
 *   g_g_g_____
 *   _r_r_r____
 * CHARGE_MODE_RETRY_WAIT
 *   Long rapid red  blink the number of retries then pause
 *   1st retry
 *     r______
 *   2nd retry
 *     r_r______
 *   5th retry
 *     r_r_r_r_r______
 *   nth retry, where n>5
 *     r_r_r_r_r_r_(pause to rearest second)
 * CHARGE_MODE_OFF_ERROR
 *   Solid red
 *   rrrrrrrrrr
 **/
void calculate_lights(struct Process* process)
{
  struct Inputs   *inputs   = &process->inputs;
  struct Outputs  *outputs  = &process->outputs;
  struct Settings *settings = &process->settings;
  
  // Increment position every 'cyclesPerStep'
  indicatorCounterRaw++;
  if (indicatorCounterRaw >= SETTINGS_LED_INDICATOR_FREQ) {
    indicatorCounterRaw = 0;
    indicatorPosition++;
    indicatorPosition = indicatorPosition%LED_BLINK_PERIOD;
  }
  
  switch(outputs->charge_mode) {
    case CHARGE_MODE_OFF: 
      outputs->led_green  = LED_ON;
      outputs->led_red    = LED_OFF;
      break;
    case CHARGE_MODE_CONSTANT_CURRENT:
      if (
        indicatorPosition == 0 ||
        indicatorPosition == 2 ||
        indicatorPosition == 4
      ) {
        outputs->led_green  = LED_ON;
      }
      else
      {
        outputs->led_green  = LED_OFF;
      }
      outputs->led_red    = LED_OFF;
      break;
    case CHARGE_MODE_CONSTANT_VOLTAGE:
      if ( 
        indicatorPosition == 0 ||
        indicatorPosition == 1 ||
        indicatorPosition == 4 ||
        indicatorPosition == 5
      ) {
        outputs->led_green  = LED_ON;
      }
      else
      {
        outputs->led_green  = LED_OFF;
      }
      outputs->led_red    = LED_OFF;
      break;
    case CHARGE_MODE_BALANCE:
      if ( 
        indicatorPosition < LED_BLINK_PERIOD/2
      ) {
        outputs->led_green  = LED_ON;
      }
      else
      {
        outputs->led_green  = LED_OFF;
      }
      outputs->led_red    = LED_OFF;
      break;
    case CHARGE_MODE_SPECIAL:
      if (
        indicatorPosition == 1 ||
        indicatorPosition == 3 ||
        indicatorPosition == 5
      ) {
        outputs->led_green  = LED_ON;
        outputs->led_red    = LED_OFF;
      }
      else if (
        indicatorPosition == 2 ||
        indicatorPosition == 4 ||
        indicatorPosition == 6
      ) {
        outputs->led_green  = LED_OFF;
        outputs->led_red    = LED_ON;
      }
      else
      {
        outputs->led_green  = LED_OFF;
        outputs->led_red    = LED_OFF;
      }   
      break;
    case CHARGE_MODE_RETRY_WAIT:
      // position is even, and less than retries*2
      // Works for maximum retries <= 5 (2* period)
      if ( 
        indicatorPosition%2 == 0 &&
        indicatorPosition < (outputs->charge_retries*2)
      ) {
        outputs->led_red    = LED_ON;
      }
      else
      {
        outputs->led_red    = LED_OFF;        
      }
      outputs->led_green  = LED_OFF;
      break;
    case CHARGE_MODE_OFF_ERROR:
      outputs->led_green  = LED_OFF;
      outputs->led_red    = LED_ON;
      break;
  }    
}


/**
 * @brief Set the calculated outputs to hardware
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void set_outputs(struct Process* process)
{
  struct Outputs *outputs = &process->outputs;
  // Set PWM
  set_pwm(PWM_CHAN_A,outputs->pwm_duty);
  // Set green LED
  led_indicators_set(outputs->led_green, outputs->led_red);
}

/**
 * @brief Enable process control
 **/
void process_control_enable( void ){
  processControlEnable = 1;
}

/**
 * @brief Disable process control
 **/
void process_control_disable(void){
  processControlEnable = 0;
}

/**
 * @brief Controll the process; Charging LiFePO4 batteries
 * 
 * This is the master process to manage the inputs and outputs for managing
 * two HP 3kW PSU's in order to charge and protect a 28*3.2V@100Ah LiFePO4 
 * battery bank
 * 
 * The task is generally
 *  * Read inputs and systems state 
 *  * Make descisions on what to do based on that state
 *  * Check for out of bounds and emergency conditions
 *  * Set the outputs
 * 
 * We are aiming for the ability to manage different charging models.
 * This can be achieved through selection by different 
 * 'processes' via the 'process_number' variable of our struct.
 *  This 'process' only effects the descision stage of the process-control
 * 
 *  Process 1
 *  ---------
 *  brief Constant current charging up to a set voltage 
 * 
 *  -# we will check whether a battery is connected, and its voltage is below the cut off
 *  -# If it is then we will
 *      + Set the PWM to a minimum value
 *      + Turn on the PSU's 
 *      + Begin constant current charging via PID on current sense to PWM duty cycle
 *  -# Once the set point is reached we
 *      + Turn off the PSU's and start a 'rest' counter
 *      + If the constant current time was very small then the rest counter is very small.w 
 *      + In the future we could record the Ah and time etc
 *  -# Once the 'rest' counter has timed out we can go to constant voltage float
 *      + Set the PWM to a minimum value
 *      + Turn on the PSU's 
 *      + Begin constant voltage charging via PID on voltage sense to PWM duty cycle
 *  -# At any stage the process can be killed or started with a push button.
 *  Visual aids/outputs have not been added or thought about yet. On car LED strips look pretty cool!
 * 
 * We do all this by splitting the process into the following functions:
 * @function get_state(process), Get the state of the system
 * @function calculate_outputs(process), Calculate the updates to the outputs
 * @function check_limits(process), Check that no inputs or propoesd outputs will break anything
 * @function set_outputs(process), Set the updated output states to hardware
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void process_control(struct Process *process)
{
  /**make sure only one instance of process_control is running, and is enabled **/
  if (process_control_running_flag) {
    return;
  }
  
  if (!processControlEnable){
    return;
  }
  // Process_control is running
  process_control_running_flag = 1;
  
  /**First things first, lets get the system state **/
  get_state(process);         // That was easy!
  
  /** Now lets calculate how to change things to maintain the process **/
  calculate_outputs(process);
  
  /** Lets check that nothing will go wrong if we use these settings **/
  check_limits(process);
  
  /** calculate the indicator LED's **/
  calculate_lights(process);
  
  /** Now we set the proposed updates to hardware **/
  set_outputs(process);
  
  //Process_control has finished
  process_control_running_flag = 0;
}
