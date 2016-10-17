LiFePO4 Charge Controller
=========================

This is a rolling project to create a fully automatic
_LiFePO4_ battery charger for my, and possibly other 
battery packs based on the HP Blade power supply ESP-120.

Principal of operation
----------------------
LiFePO4 batteries have a relativly simple charging profile:

 * Charge at constant current up to a set voltage, usually 3.65Volts per cell
 * Turn off charging and rest the batteries for a given time
 * If you must, float the batteries at 3.45Vpc
 
Thanks to the efforts of a German group the ESP-120 is well
documented with a known and stable voltage control from 30-53V
with positive voltage into a trim-pot pin and 53-63 with a slight
negative voltage to this pin. There is an inverse relationship
between voltage at the pot, and the supplies output voltage.
~5Vin=> 30Vout, 0Vin=>53Vout.

To operate as a charger two of these units are connected
in series. This is easy as the DC output is isolated from
the case. To achieve constant current charging, the voltage of the
supplies is brought up slowly by modifying (decreasing) Vin 
whilst monitoring the current until the desired current is reached.
Vin is decreased to maintain this constant current until There
setpoint is reached, at which point the PSU's are turned off.

The PSU's are protected by a very large external diode (75A, 200V).

The planned itterations are as follows, with each mature
and completed iteration being moved into its appropriate 
folder upon completion:

Repo layout
-----------
```
.
├── code            Charge controller logic
│   └── AVR-lib     General AVR libraries. Should be a submodule
│       └── lib
├── doc             Documents relating to the ESP-120/HP3KW
├── pcb             Output PCB files
└── schematic       Schematic
```

The code is written in C using [Kdevelop] and built using [avr-gcc].

The schematic and PCB layout are done in [Kicad]

Current Implimentation
----------------------
Photos to come


Iterations:
------------
1.  **Mark I: Proof of concept using onboard**

    - Construction on breadboard
    - Powered off the PSU power GND - 12V via an LM7805, so has common ground with Cycle Analyst
    - shunt and differential opamp via built in 10bit ADC for current measurement
    - 100:1 resistor divider via 10bit ADC for output voltage measurement
    - Output is a PWM based 'DAC' to a fet/transistor between two pot's setting max/min voltage.
    - Output/input via isolated serial interface.
  
2.  **Mark II: further improvements**
    - Impliment isolated I2C to PSU controllers for PSON and current measurement.
    - Check PSU current VS shunt current, once verified they are the same with similar reliability, discontinue use of shunt.
    - Impliment software usart for Rx only to recieve SOC etc. from the cycle analyst
    - Assess whether external DAC's and ADC's are nescesarry.
    - Likely that external ADC will be, so use it.
    - Investigate and impliment different charging strategies.
    - LCD?
    - BMS integration
  
3.  **Mark III: Close to Final** 
    - PCB based, on its own board. 
    - Use specialised AVR with CAN support
    - Similar codebase to MarkII, trying to remove interrupt driven routines
    - Safety/Failsafe etc. much more error checking.
    - Impliment CAN somehow

Currently half way through Iteration 2. Soon BMS integration will be required, and maybe we will do CAN bus things then too. Maybe port to STM32.


### Mark I: Proof of concept using onboard, Design considerations

Inputs:
-------
 **Current:** 
   Current will be measured using the shunt resistor on the PSU's PCB.
   It has a resistance of 0.0002Ohm, and will require a diffrential opamp
   in order to amplify the signal to the 5V range. There is already an opamp
   on the PCB that does this, and it may be sufficient to use this.
   
   One 10bit ADC channel on the micro will do this, preferebly with the 
   1.25V internal reference.
   
   With a maximum current of say 60A, the finest measurement we can make is:
     60/1024 = 59mA
   which is sufficient.
   
 **Output Voltage**
   Output volatage will be measured by another 10bit ADC channel via a 100k:1k 
   resistor divider network.
   Using the internal 1.25 voltage reference, this gives a maximum measurable voltage of
   125V with an accuracy of 0.122, again, more than sufficient!
   (DAC RC calculator)[http://sim.okawa-denshi.jp/en/PWMtool.php]
   
Outputs:
--------
 **Vin:**
   Volatage between power ground and a pad from a reomoved potentiometer needs to 
   be adjusted from ~5V to 0V.
   
   In this implimentation we will use PWM driving a fet to ground on from the 5V supply rail
   with the pot pin in the middle, and a 330nF cap to ground.
   
 **Serial:**
   Debugging and logging will be via serial
   
 **LED's and stuff:**
   - Blinking light (things alive)
   - Charging
   - Complete
   - State of charge bar graph for in the window 8)
   - Even better would be EL wire or tape on the exterior.

[Kdevelop]: https://www.kdevelop.org/
[avr-gcc]: http://www.nongnu.org/avr-libc/
[kicad]: http://kicad-pcb.org/