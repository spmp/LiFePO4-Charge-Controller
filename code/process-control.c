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
 * @brief Variables that control whether the process_control is able to run (again)
 * 
 * @var begin_process_control_flag
 * Flag that indicates that process should run
 * @var process_control_running_flag
 * Flag that indicates process_control is running
 * @var process_control_enable
 * Flag for global enable/disable of process_control
 **/
uint8_t PIDtype = 0;
uint16_t cPIDmaxread = 30000;

uint8_t begin_process_control_flag = 0;
uint8_t process_control_running_flag = 0;
uint8_t processControlEnable = 0;               //disabled by default

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
 * @def SETPOINT_REACHED_COUNT
 * How many counts of setpoint_reached_counter to count for
 * @def PWM_THRESHOLD_REDUCTION
 * How much to reduce the PWM by if we enounter an over threshold
 **/
#define SETPOINT_REACHED_COUNT      50
#define PWM_THRESHOLD_REDUCTION     1000
uint8_t setpoint_reached_counter = 0;
uint8_t rest_init_flag = 0;
uint8_t done_init_flag = 0;
uint8_t startPSUflag = 0;
uint32_t Ah_QuarterSecond = 0;

/** @brief struct(s) for PID **/
struct u_PID_DATA pidData_cc;    // PID data for constant current
struct u_PID_DATA pidData_cv;    // PID data for constant voltage
// = {   // PID data for constant voltage

/** @brief struct PSU_state psu_state for storing PSU data **/
struct PSU_state psu_state = {0};

/** @brief struct Process process for program 1 **/
struct Process process = {
    {0},
    {
    .pwm_duty = PWM_START,          // Duty cycle of pwm_duty
    .PSU_state = 20,         // State of the PSU's
    .charge_state = 0,      // State of charging, Bulk, rest, float, Done
    .error_code = 0,         // Code of error that is encounted, will be set and cleared regularily - needs to be writtne to EEPROM somewhere
    .charge_progress = 0    // Percentage of charging done.
    },
    {
    .program_number = 1,     // An index for the process, such that process_control knows with process to run.
    .current_max = 35000,     // Absolute maximum current. Shutdown if over
    .current_threhold = 3500,// Current at which to try to recover over current condition, needs to be lower than current_max and above current_charge
    .current_charge = 30000, // Current to charge the batteries at
    .current_float = 20000, // Max current for float charge
    .voltage_max = 1037,    // Absolute maximum battery voltage. Shutdown if over
    .voltage_min = 8300,       // Minimum battery voltage. Error or batteries disconnected if under.
    .voltage_threshold = 1035, // Voltage at which to try to recover over voltage condition, needs to be lower than voltage_max and above voltage_charged
    .voltage_charged = 1013, // Voltage at which to stop bulk charging
    .voltage_float = 942,   // Voltage at which to float the batteries
    .rest_time = 36,         // Time between charged voltage and driving or float/done. This is in seconds per Ah
    .max_PSU_temp = 70,      // Maximum temperature for any PSU before shutdown
    .max_PSU_volt = 300,     // Maximum PSU line voltage
    .min_PSU_volt = 200,     // Minimum PSU voltage
    .max_battery_temp = 50,  // Maximum temperature of any battery before shutdown.
    /** PID **/
    .PIDoutput = 0,          // Output from the PID algorythm
    .cc_P_Factor = 1,        //! The cc Proportional tuning constant, multiplied with SCALING_FACTOR
    .cc_I_Factor = 0,        //! The cc Integral tuning constant, multiplied with SCALING_FACTOR
    .cc_D_Factor = 0,        //! The cc Derivative tuning constant, multiplied with SCALING_FACTOR
    .cv_P_Factor = 3,        //! The cv Proportional tuning constant, multiplied with SCALING_FACTOR
    .cv_I_Factor = 0,        //! The cv Integral tuning constant, multiplied with SCALING_FACTOR
    .cv_D_Factor = 3         //! The cv Derivative tuning constant, multiplied with SCALING_FACTOR
    }
};

/** 
 * @brief Initialise the PID from Process struct
 * @param *process, a struct of type Process in which get the pid initialisation variables
 **/

/**
 * @brief Get the state of the system, input values and output states
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void get_state(struct Process *process) {
    struct Inputs *inputs = &process->inputs;
    struct Outputs *outputs = &process->outputs;
    
    // Get the external states
    get_psu_state(inputs);
    inputs->current = (double)inputs->PSU1AnalogData.Current*1.2057067147 + 1811.45;
    inputs->voltage = get_voltage();
    
    
}

/**
 * @brief Calculate the outputs in order to controll the process
 * 
 * Charge at constant (high) current to constant voltage until current is less than
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void calculate_outputs(struct Process* process)
{

  struct Inputs *inputs = &process->inputs;
  struct Outputs *outputs = &process->outputs;
  struct Settings *settings = &process->settings;
  
  // Change PID etc depending on PIDtype
  switch(PIDtype) {
      
      case 1: //Constant Current PID
          //Check if the voltage set point has been reached.
          if (inputs->voltage >= settings->voltage_charged ){
              //TODO: Impliment a decriment of this counter ie.. setpoint+5, then in the main body setpoint- such that the setpoints must me reached sequentially.
              setpoint_reached_counter++;
              if (setpoint_reached_counter >= SETPOINT_REACHED_COUNT )
              {
                  //Turn off the PSU's
                  outputs->pwm_duty = PWM_START;
                  settings->PIDoutput = 0;
                  psu_power(3,0);
                  startPSUflag = 0;
                  // Calculate how long to wait.
                  outputs->rest_timer = timestamp*2;
                  outputs->float_timer = timestamp*3;
                  //Reset setpoint counter
                  setpoint_reached_counter = 0;
                  //FInish Cahrging here
                  //PIDtype = 0;
                  //Change to Waiting
                  PIDtype = 2;
                  break;
              }
          }
          // Setpoint is not reached.
          else 
              //If the PSU's are not on/started they must be started/turned-on.
              if(!startPSUflag)
              {
                  //Start the PSU
                  outputs->pwm_duty = PWM_START;
                  psu_power(3,1);
                  startPSUflag = 1;
                  _delay_ms(500);
              }
              //Constant current PID
              settings->PIDoutput = pid_proportional_current( inputs->current, settings->current_charge, 300, 25);
          break;
          
      case 2:
          //wait
          //TODO: Change light blinking pattern
          if (timestamp >= outputs->rest_timer)
          {
              PIDtype =3;
          }
          break;
          
      case 3:
          //Constant Voltage PID
          if (timestamp >= outputs->float_timer)
          {
              //ensure output returned to safe state
              outputs->pwm_duty = PWM_START;
              settings->PIDoutput = 0;
              //Turn off the PSU's
              psu_power(3,0);
              startPSUflag = 0;
              //Change to Done..
              PIDtype = 0;
              break;
          }
          else
              if(!startPSUflag)
              {
                  //Start the PSU
                  outputs->pwm_duty = PWM_START;
                  outputs->pwm_duty = PWM_START;
                  psu_power(3,1);
                  startPSUflag = 1;
                  _delay_ms(1000);
              }
              if( inputs->current >= settings->current_float )
              {
                  settings->PIDoutput = pid_proportional_current( inputs->current, settings->current_float, 300, 25);
              }
              else
              {
                  settings->PIDoutput = pid_proportional_max( inputs->voltage, settings->voltage_float, 1100, 0xFFF, 0.25, 1, 30);
              }
          break;
      default:
          //Turn off the PSU's
          outputs->pwm_duty = PWM_START;
          settings->PIDoutput = 0;
          psu_power(3,0);
          startPSUflag = 0;
          PIDtype = 0;
  }
    
    }
 
    
    //Check PWM output --> Inverted (ie high = low output so use -= )
    outputs->pwm_duty -= settings->PIDoutput;
    //settings->PIDoutput = 0;
    // Maintain PWM Limits/Boundaries
    if (outputs->pwm_duty >= PWM_TOP-1){
        outputs->pwm_duty = PWM_TOP-2;
    }
    if (outputs->pwm_duty <= 0){
        outputs->pwm_duty = 0;
    }
}

/**
 * @brief Check that no inputs or propoesd outputs will break anything
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void check_limits(struct Process* process)
{
    struct Inputs *inputs = &process->inputs;
    struct Outputs *outputs = &process->outputs;
    struct Settings *settings = &process->settings;
    
    //Check if any of the stop conditions are reached.
    if((inputs->current >= settings->current_max) || (inputs->voltage >= settings->voltage_max))
    {
        //ensure output returned to safe state
        outputs->pwm_duty = PWM_START;
        settings->PIDoutput = 0;
        //Turn off the PSU's
        psu_power(3,0);
        startPSUflag = 0;
        //Change to Done..
        PIDtype = 0;
        send_string_p(PSTR("A stop condition was breached. Shutting down.\r\n"));
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
    set_pwm(PWM_CHAN_A,outputs->pwm_duty);
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
    
    /** Now we set the proposed updates to hardware **/
    set_outputs(process);
    
    /** Lets check that nothing has gone wrong or will go wrong if we use these settings **/
    check_limits(process);
    
    //Process_control has finished
    process_control_running_flag = 0;
}
