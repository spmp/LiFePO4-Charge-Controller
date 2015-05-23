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
    .Ah_count = 30,          // How many Ah's have passed through the charger in this cycle
    .charge_timer = 0,      // How long have we been in the 'bulk' charging phase
    .cur_rest_time = 0,     // How long have we been resting?
    .rest_timer = 0,        // How long do we need to rest for?
    .float_timer = 0,        // How long do we need to float charge?
    .charge_state = 0,      // State of charging, Bulk, rest, float, Done
    .error_code = 0,         // Code of error that is encounted, will be set and cleared regularily - needs to be writtne to EEPROM somewhere
    .charge_progress = 0    // Percentage of charging done.
    },
    {
    .program_number = 1,     // An index for the process, such that process_control knows with process to run.
    .current_max = 40000,     // Absolute maximum current. Shutdown if over
    .current_threhold = 3500,// Current at which to try to recover over current condition, needs to be lower than current_max and above current_charge
    .current_charge = 30000, // Current to charge the batteries at
    .voltage_max = 1037,    // Absolute maximum battery voltage. Shutdown if over
    .voltage_min = 8300,       // Minimum battery voltage. Error or batteries disconnected if under.
    .voltage_threshold = 1035, // Voltage at which to try to recover over voltage condition, needs to be lower than voltage_max and above voltage_charged
    .voltage_charged = 1012, // Voltage at which to stop bulk charging
    .voltage_float = 976,   // Voltage at which to float the batteries
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
void init_PID(struct Process *process)
{
    struct Settings *settings = &process->settings;
    u_pid_Init(settings->cc_P_Factor * SCALING_FACTOR, settings->cc_I_Factor * SCALING_FACTOR, settings->cc_D_Factor * SCALING_FACTOR, &pidData_cc);
    u_pid_Init(settings->cv_P_Factor * SCALING_FACTOR, settings->cv_I_Factor * SCALING_FACTOR, settings->cv_D_Factor * SCALING_FACTOR, &pidData_cv);
}

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
    
    //Calculate Amp Hours assuming readings every 4th of a second
    Ah_QuarterSecond += inputs->current;
    outputs->Ah_count += Ah_QuarterSecond/(6*6*4);
    
// //     inputs->current = (psu_state.PSU1_current+psu_state.PSU1_current)/2;
    
    // Get the internal states
// //     outputs->pwm_duty = get_pwm_duty(PWM_CHAN_A, ABSOLUTE);
//     outputs->charge_progress = get_SoC(ABSOLUTE);
}

/**
 * @brief Calculate the outputs in order to controll the process
 * 
 * Details are elsewhere and in code.
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void calculate_outputs(struct Process* process)
{
    
    struct Inputs *inputs = &process->inputs;
    struct Outputs *outputs = &process->outputs;
    struct Settings *settings = &process->settings;
   
//     outputs->Ah_count = u_pid_Controller(settings->voltage_charged, inputs->voltage, &pidData_cv);
//     outputs->pwm_duty += (int32_t)(outputs->Ah_count/8);
//      u_pid_Controller(settings->voltage_charged, inputs->voltage, &pidData_cv);
//     pid_proportional( int32_t Measured, int32_t setPoint, int32_t MeasuredMax, int32_t OutputMax, float Proportionality, uint8_t Power);
    
/*
 * The process-control is primarily calculating outputs. This process is broken up as follows:
 *   Check if any maximum conditions (current and voltage) are breached and shutdown if so.
 *     The `PIDtype' sets the stage of the charging and is inspected by the switch/case statement.
 *     The `PIDtype's are:
 *       0 Stop. Stop charging, power off PSU's
 *       1 Constant current (startup default). Bulk charging at settings->current_charge, until settings->voltage_charged is reached
 *       2 Absorbtion. This stage waits for a period of time defined by the battery manufacturer
 *       3 Constant voltage: A constant voltage is maintained for a further time period.
 */
    if( (inputs->voltage >= settings->voltage_max) || (inputs->current >= settings->current_max)) {
//             send_string_p(PSTR("OUTputs out of bounds, quitting PID/r/n"));
            settings->PIDoutput = PWM_START;
            //Turn off the PSU's
            psu_power(3,0);
            PIDtype = 0;
        }
        
    else {
           
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
                        psu_power(3,0);
                        startPSUflag = 0;
                        // Calculate how long to wait.
                        outputs->rest_timer = timestamp*2;
                        outputs->float_timer = timestamp*3;
                        //Reset setpoint counter
                        setpoint_reached_counter = 0;
                        //FInish Cahrging here
                        PIDtype = 0;
                        //Change to Waiting
                        //PIDtype = 2;
                        
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
                if (timestamp >= outputs->charge_timer)
                    PIDtype =3;
                break;
                
            case 3:
                //Constant Voltage PID
                if (timestamp >= outputs->float_timer)
                {
                    //ensure output returned to safe state
                    outputs->pwm_duty = PWM_START;
                    //Turn off the PSU's
                    psu_power(3,0);
                    startPSUflag = 0;
                    //Change to Done..
                    PIDtype = 0;
                }
                else
                    if(!startPSUflag)
                    {
                        //Start the PSU
                        outputs->pwm_duty = PWM_START;
                        psu_power(3,1);
                        startPSUflag = 1;
                        _delay_ms(500);
                    }
                    settings->PIDoutput = pid_proportional( inputs->voltage, settings->voltage_float, 1100, 0xFFF, 1, 1);
                break;
            default:
                //Turn off the PSU's
                outputs->pwm_duty = PWM_START;
                psu_power(3,0);
                startPSUflag = 0;
        }
    
    }
 
    
    //Check PWM output --> Inverted (ie high = low output so use -= )
    outputs->pwm_duty -= settings->PIDoutput;
    // Maintain PWM Limits/Boundaries
    if (outputs->pwm_duty >= PWM_TOP-1){
        outputs->pwm_duty = PWM_TOP-2;
    }
    if (outputs->pwm_duty <= 0){
        outputs->pwm_duty = 0;
    }
//     if (outputs->pwm_duty >= 550){
//         outputs->pwm_duty = 550;
//     }
    
// // // //     /** The actual process control is as follows: **/
// // // //     
// // // //     /** Which 'program' are we running? **/
// // // //     switch(settings->program_number) {
// // // //         
// // // //         //We only have one program at the moment, so as a catchall lets do:
// // // //         default:
// // // //             /** Where are we in the charging cycle? **/
// // // //             switch(outputs->charge_state) {
// // // //                 // Not charging
// // // //                 case 0:
// // // //                     /** 
// // // //                      * @brief Transition from not charging to charging
// // // //                      * 
// // // //                      * We assume that we are not charging because the process
// // // //                      * has just begun in which case we must make sure that it is
// // // //                      * safe to begin to charge as such we need to:
// // // //                      * -# Check the battery voltage
// // // //                      *  - if it is high, then charging is done
// // // //                      *  - if it low then there is an error and we need to stop immediatly
// // // //                      * -# Reduce the ouput signal to a safe level
// // // //                      * -# Check the state of the PSU's
// // // //                      *  - Temperature
// // // //                      *  - Line voltage
// // // //                      *  If all is fine progress, otherwie fire an error and stop
// // // //                      * -# Power on the PSU's
// // // //                      *  - Check status, if all is fine progress, or error otherwise
// // // //                      * -# Set the charge_state to bulk
// // // //                      * NOTE no need to break at the end of any of these routines!
// // // //                      **/
// // // //                     if (inputs->voltage >= settings->voltage_float) {
// // // //                         //Batteries are already charged
// // // //                         outputs->charge_state = 4;      //Charging done
// // // //                         break;
// // // //                     }
// // // //                     if (inputs->voltage <= settings->voltage_min) {
// // // //                         //Batteries are not connected or there is  problem
// // // //                         outputs->charge_state = 7;      //Charging error
// // // //                         outputs->error_code = 1;        //Battery undervolt!
// // // //                         break;
// // // //                     }
// // // //                         //Set PWM to minimum
// // // //                     set_pwm(PWM_CHAN_A, PWM_START);
// // // //                     if (get_pwm_duty(PWM_CHAN_A,ABSOLUTE) != PWM_START) {
// // // //                         //PWM failed to be set so the process is doomed
// // // //                         outputs->charge_state = 7;      //Charging error
// // // //                         outputs->error_code = 2;        //PWM Set Error
// // // //                         break;
// // // //                     }
// // // //                         //Check PSU states
// // // //                     if ( psu_state.PSU1_temperature >= settings->max_PSU_temp || psu_state.PSU2_temperature >= settings->max_PSU_temp ) {
// // // //                         //One of the PSU's is over temperatur
// // // //                         outputs->charge_state = 7;      //Charging error
// // // //                         outputs->error_code = 3;        //PSU temperature error
// // // //                         break;
// // // //                     }
// // // //                     if (psu_state.PSU1_line_voltage >= settings->max_PSU_volt || psu_state.PSU1_line_voltage < settings->min_PSU_volt || psu_state.PSU2_line_voltage >= settings->max_PSU_volt || psu_state.PSU2_line_voltage < settings->min_PSU_volt) {
// // // //                         //One of the PSU's is over or under voltage
// // // //                         outputs->charge_state = 7;      //Charging error
// // // //                         outputs->error_code = 4;        //PSU voltage error
// // // //                         break;
// // // //                     }
// // // //                     //Turn on the PSU's
// // // //                     psu_power(1,1);
// // // //                     psu_power(2,1);
// // // //                     _delay_ms(5);                       //Delay as we wait for the power supplies to turn on
// // // //                     if (!psu_power_check(1) || !psu_power_check(2)) {
// // // //                         //Something went wrong! PSU's have not powered on
// // // //                         outputs->charge_state = 7;      //Charging error
// // // //                         outputs->error_code = 5;        //PSU PSON error
// // // //                         break;
// // // //                     }
// // // //                     
// // // //                         // OK, its all done, we can progress!
// // // //                     outputs->charge_state = 2;
// // // // 
// // // //                         
// // // //                 //Bulk charging
// // // //                 case 1:
// // // //                     /**
// // // //                      * @brief Manage the bulk charging, this is constant current (cc) up to a setpoint
// // // //                      * 
// // // //                      * We assume that the PSU's are on and everything is OK
// // // //                      * -# Check voltage
// // // //                      *  - If low progress
// // // //                      *  - If at or above setpoint add to setpoint reached counter
// // // //                      *  - If the setpoint counter is big enough increase charge_state to Rest
// // // //                      * -# Run PID for cc control to setpoint 
// // // //                      * -# check for current and voltage above _threshold
// // // //                      *  - if so apply standard reduction to current control parameter
// // // //                      * -# incriment Ah
// // // //                      **/
// // // //                     // Check voltages
// // // //                     if (inputs->voltage >= settings->voltage_charged){
// // // //                         setpoint_reached_counter++;
// // // //                         // Has charging really finished?
// // // //                         if (setpoint_reached_counter >= SETPOINT_REACHED_COUNT ) {
// // // //                             // OMG we are done bulk charging
// // // //                             outputs->charge_state = 2;
// // // //                         }
// // // //                     }
// // // //                     
// // // //                     /** 
// // // //                      * @brief This is where most of the MAGIC happens, and where things can go horrible wrong!
// // // //                      * 
// // // //                      * PID PID PID PID
// // // //                      * Personally I think that we CANNOT use the I term due to the cumulative error as the chargers are below the battery voltage
// // // //                      *  Divisors, gain terms, oscillation!
// // // //                      * 
// // // //                      * Another approach may be to have a set voltage or current in which we kick over into another PID mode, like low and high so that we can use I to reduce oscillation.
// // // //                      **/
// // // //                     outputs->pwm_duty -= u_pid_Controller(settings->current_charge, inputs->current, &pidData_cc)/4;
// // // //                     
// // // //                     // Check if over _thresholds
// // // //                     if ( inputs->current >= settings->current_threhold || inputs->voltage >= settings->voltage_threshold ) {
// // // //                         //OMG reduce the PWM!
// // // //                         outputs->pwm_duty += PWM_THRESHOLD_REDUCTION;
// // // //                     }
// // // //                     
// // // //                     // Incdriment Ah
// // // //                     outputs->Ah_count += inputs->current;
// // // //                     
// // // //                 //Rest
// // // //                 case 2:
// // // //                     /**
// // // //                      * @brief Intiate a rest time based on the Ah into the battery
// // // //                      * 
// // // //                      * We assume that bulk charging is complete
// // // //                      * -# Run once on rest
// // // //                      *  - gracefully reduce PWM
// // // //                      *  - Power down the PSU's (checking they do)
// // // //                      *  - Initialise rest counter (Will be large)
// // // //                      * -# reduce rest counter, possibly every seconds
// // // //                      * -# When rest counter is done progress to Float charge
// // // //                      **/
// // // //                     // Do stuff on the first running
// // // //                     if (!rest_init_flag) {
// // // //                         //Power down the PSU's
// // // //                         outputs->pwm_duty = PWM_START;
// // // //                         psu_power(1,0);
// // // //                         psu_power(2,0);
// // // //                         
// // // //                         /**
// // // //                          * @brief  Calculate rest time in _medium_ time periods of a seconds
// // // //                          *  Batteries specified as 1 hour per 100Ah.
// // // //                          * Which is 1*(60*60/100) = 36 seconds per Ah.
// // // //                          * Which is 180 _medium_ timesteps per Ah.
// // // //                          * Now we are storing Ah as Amps . 1/5th of a second
// // // //                          * So want seconds so we need to divide Ah by 180
// // // //                          *  MY MATH IS ALL WRONG!
// // // //                          **/
// // // //                         outputs->rest_timer = outputs->Ah_count / settings->rest_time ;
// // // //                         
// // // //                         //Set the rest_init_flag so we dont do this again
// // // //                         rest_init_flag = 1;
// // // //                     }
// // // //                     
// // // //                     //Decriment the rest timer
// // // //                     outputs->rest_timer --;
// // // //                     
// // // //                     if (!outputs->rest_timer) {
// // // //                         //Done resting
// // // //                         outputs->charge_state = 3;
// // // //                     }
// // // //                         
// // // //                 //Constant voltage
// // // //                 case 3:
// // // //                     /** 
// // // //                      * @brief Constant voltage charging
// // // //                      * 
// // // //                      * We assume that bulk and rest charging has been done.
// // // //                      * -# set PWM output low, and go through the same startup sequence as in Bulk
// // // //                      * -# Check current, it must be below current setpoint
// // // //                      *  - What to do if it is not?
// // // //                      * -# Check voltage limits
// // // //                      *  - Do stuff
// // // //                      * -# Run PID on voltage
// // // //                      * -# Check for voltage above voltage_charged
// // // //                      *  - Take action if it is over
// // // //                      * -# @todo: How do we determine when float should finish?
// // // //                      **/
// // // //                         //Set PWM to minimum
// // // //                     set_pwm(PWM_CHAN_A,PWM_START);
// // // //                     if (get_pwm_duty(PWM_CHAN_A,ABSOLUTE) != PWM_START) {
// // // //                         //PWM failed to be set so the process is doomed
// // // //                         outputs->charge_state = 7;      //Charging error
// // // //                         outputs->error_code = 2;        //PWM Set Error
// // // //                         break;
// // // //                     }
// // // //                         //Check PSU states
// // // //                     if ( psu_state.PSU1_temperature >= settings->max_PSU_temp || psu_state.PSU2_temperature >= settings->max_PSU_temp ) {
// // // //                         //One of the PSU's is over temperatur
// // // //                         outputs->charge_state = 7;      //Charging error
// // // //                         outputs->error_code = 3;        //PSU temperature error
// // // //                         break;
// // // //                     }
// // // //                     if (psu_state.PSU1_line_voltage >= settings->max_PSU_volt || psu_state.PSU1_line_voltage < settings->min_PSU_volt || psu_state.PSU2_line_voltage >= settings->max_PSU_volt || psu_state.PSU2_line_voltage < settings->min_PSU_volt) {
// // // //                         //One of the PSU's is over or under voltage
// // // //                         outputs->charge_state = 7;      //Charging error
// // // //                         outputs->error_code = 4;        //PSU voltage error
// // // //                         break;
// // // //                     }
// // // //                     //Turn on the PSU's
// // // //                     psu_power(1,1);
// // // //                     psu_power(2,1);
// // // //                     _delay_ms(5);                       //Delay as we wait for the power supplies to turn on
// // // //                     if (!psu_power_check(1) || !psu_power_check(2)) {
// // // //                         //Something went wrong! PSU's have not powered on
// // // //                         outputs->charge_state = 7;      //Charging error
// // // //                         outputs->error_code = 5;        //PSU PSON error
// // // //                         break;
// // // //                     }
// // // //                     
// // // //                     //All good, now we can do Float charging for... ever...
// // // //                     
// // // //                     /** 
// // // //                      * @brief This is where most of the MAGIC happens, and where things can go horrible wrong!
// // // //                      * 
// // // //                      * PID PID PID PID
// // // //                      * Personally I think that we CANNOT use the I term due to the cumulative error as the chargers are below the battery voltage
// // // //                      *  Divisors, gain terms, oscillation!
// // // //                      * 
// // // //                      * Another approach may be to have a set voltage or current in which we kick over into another PID mode, like low and high so that we can use I to reduce oscillation.
// // // //                      **/
// // // //                     outputs->pwm_duty -= u_pid_Controller(settings->voltage_float, inputs->voltage, &pidData_cv)/4;
// // // //                     
// // // //                     // Check if over _thresholds
// // // //                     if ( inputs->current >= settings->current_threhold || inputs->voltage >= settings->voltage_threshold ) {
// // // //                         //OMG reduce the PWM!
// // // //                         outputs->pwm_duty += PWM_THRESHOLD_REDUCTION;
// // // //                     }
// // // //                     
// // // //                     // Incdriment Ah
// // // //                     outputs->Ah_count += inputs->current;
// // // //                     
// // // //                     
// // // //                 // Done
// // // //                 case 4:
// // // //                     /**
// // // //                      * @brief the charging is done or has been prematurely stopped.
// // // //                      * 
// // // //                      * We assume nothing except we need charging to stop fairly rapidly
// // // //                      * -# Reduce PWM
// // // //                      * -# Power off PSU's
// // // //                      * -# Reset all counters - NO we will do that on start!
// // // //                      * -# Make notes or whatever. - Whatever!
// // // //                      **/
// // // //                     // Do stuff on the first running
// // // //                     if (!done_init_flag) {
// // // //                         //Power down the PSU's
// // // //                         outputs->pwm_duty = PWM_START;
// // // //                         psu_power(1,0);
// // // //                         psu_power(2,0);
// // // //                     
// // // //                         //We are done, just uselessly loop through from now on
// // // //                         done_init_flag = 1;
// // // //                     }
// // // //                     
// // // //                     break;
// // // //             }
// // // //     
// // // //     }
}

/**
 * @brief Check that no inputs or propoesd outputs will break anything
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void check_limits(struct Process* process)
{
    //
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
    
    /** Lets check that nothing has gone wrong or will go wrong if we use these settings **/
//     check_limits(process);
    
    /** Now we set the proposed updates to hardware **/
    set_outputs(process);
    
    //Process_control has finished
    process_control_running_flag = 0;
}
