/*
 *  Kennyth Kouch
 *  Fall 2022
 *  Section 3
 *  12/2/2022
 *  Lab #10
 *  Lab #10: Robot Navigation
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

Timer_A_PWMConfig  timerPWMConfig;
uint16_t leftPeriod = 0, rightPeriod = 0;
uint16_t leftDivider = 20, rightDivider = 20;
uint16_t leftDuty = 30, rightDuty = 30;
#define LEFTCHANNEL TIMER_A_CAPTURECOMPARE_REGISTER_4
#define RIGHTCHANNEL TIMER_A_CAPTURECOMPARE_REGISTER_3
uint8_t leftSensors = 0, rightSensors = 0;
uint8_t lostTimer = 0;
enum {onlyLeft, moreLeft, Equal, moreRight, onlyRight, None} sensorState;
enum {standBy,Track,Lost} robotState;
enum {Forward,Reverse,Stay} motorState;
uint8_t s[8] = {0,0,0,0,0,0,0,0};

//Function Prototypes
void config432IO();
void configRobotIO();
void readSensors();
void processSensors();
void toggleLEDs();
void ledStates();
void bumperSwitchesHandler();
void configMotors();
void configPWMTimer(uint16_t clockPeriod, uint16_t clockDivider, uint16_t duty,
                    uint16_t);

/*
 * Name: main
 * Description: main
 * Author: Kennyth Kouch
 * Inputs: none
 * Outputs: none
 */
int main(void)
{
    MAP_WDT_A_holdTimer();

    config432IO();
    configRobotIO();
    configMotors();

    uint8_t i = 0;
    for(i = 0; i < 2; i++) toggleLEDs();

    robotState = standBy;
    sensorState = None;
    motorState = Stay;
    configPWMTimer(leftPeriod,leftDivider,leftDuty, LEFTCHANNEL);
    configPWMTimer(rightPeriod,rightDivider,rightDuty, RIGHTCHANNEL);

    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    __enable_interrupts();

    while(1)
    {
        readSensors();
        processSensors();
        configMotors();
        ledStates();
        __delay_cycles(25000);
    }
}

/*
 * Name: configIO
 * Description: Configures inputs/outputs
 * Author: Kennyth Kouch
 * Inputs: none
 * Outputs: none
 */
void config432IO()
{
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
}

/*
 * Name: configRobotIO
 * Description: Configures input/outputs of RSLK MAX
 * Author: Kennyth Kouch
 * Inputs: none
 * Outputs: none
 */
void configRobotIO()
{
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN0); //BS#0 configuration
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN0);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN0, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN0);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN2); //BS#1 configuration
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN2);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN2);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN3); //BS#2 configuration
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN3);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN3, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN3);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN5); //BS#3 configuration
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN5);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN5);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN6); //BS#4 configuration
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN6);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN6);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN7); //BS#5 configuration
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN7);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN7, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN7);

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2); // L-Wheel Enable
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0); // R-Wheel Enable
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4); // L-Wheel Direction
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN5); // R-Wheel Direction
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7); // L-Wheel Sleep
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6); // R-Wheel Sleep
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION); //L-Wheel Timer Output
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN6,GPIO_PRIMARY_MODULE_FUNCTION); // R-Wheel Timer Output

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); // L-Wheel Enable
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // R-Wheel Enable
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4); // L-Wheel Direction
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5); // R-Wheel Direction
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); // L-Wheel Sleep
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // R-Wheel Sleep
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); // L-Wheel PWM
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); // R-Wheel PWM

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0); // L-Front LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN5); // R-Front LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN6); // L-Rear LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN7); // R-Rear LED

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3); // CNTL Even
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN2); // CNTL Odd
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);

    MAP_GPIO_registerInterrupt(GPIO_PORT_P4, bumperSwitchesHandler);
}

/*
 * Name: readSensors
 * Description: Configures and reads sensors
 * Author: Kennyth Kouch
 * Inputs: none
 * Outputs: none
 */
void readSensors()
{
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3); // Turn on even & odd IRLEDs
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN2);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0); // IRLED 0
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN1); // IRLED 1
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN1);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN2); // IRLED 2
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN2);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN3); // IRLED 3
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN3);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN4); // IRLED 4
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN4);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN5); // IRLED 5
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN5);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN6); // IRLED 6
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN6);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN7); // IRLED 7
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN7);
    __delay_cycles(30);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN0);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN1);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN2);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN3);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN4);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN5);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN6);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN7);
    __delay_cycles(3000);
    s[0] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN0);
    s[1] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN1);
    s[2] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN2);
    s[3] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN3);
    s[4] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN4);
    s[5] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN5);
    s[6] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN6);
    s[7] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN7);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);
}

/*
 * Name: processSensors
 * Description: Processes sensor data
 * Author: Kennyth Kouch
 * Inputs: none
 * Outputs: none
 */
void processSensors()
{

    rightSensors = s[0] + s[1] + s[2] + s[3];
    leftSensors = s[4] + s[5] + s[6] + s[7];

    if(leftSensors > 0 && rightSensors == 0) sensorState = onlyLeft;
    else if(leftSensors > rightSensors) sensorState = moreLeft;
    else if (leftSensors == rightSensors && leftSensors != 0) sensorState = Equal;
    else if(leftSensors == 0 && rightSensors > 0) sensorState = onlyRight;
    else if(leftSensors < rightSensors) sensorState = moreRight;
    else if(leftSensors == rightSensors && leftSensors == 0) sensorState = None;
}

/*
 * Name: toggleLEDs
 * Description: Toggles all LEDs on the TI-RSLK MAX and MSP432
 * Author: Kennyth Kouch
 * Inputs: none
 * Outputs: none
 */
void toggleLEDs()
{
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0);
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN2);

    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P8, GPIO_PIN0);
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P8, GPIO_PIN5);
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P8, GPIO_PIN6);
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P8, GPIO_PIN7);

    __delay_cycles(3000000);
}

/*
 * Name: bumperSwitchesHandler
 * Description: Debounces bumper switches off TI-RSLK MAX
 * Author: Kennyth Kouch
 * Inputs: none
 * Outputs: none
 */
void bumperSwitchesHandler()
{
    uint16_t status;
    __delay_cycles(50000);
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);

    if(status == GPIO_PIN0 && robotState == standBy) robotState = Track;
    else if(status && (robotState == Track || robotState == Lost)) robotState = standBy;
}

/*
 * Name: ledStates
 * Description: Configures LEDs
 * Author: Kennyth Kouch
 * Inputs: none
 * Outputs: none
 */
void ledStates()
{
    switch(robotState)
    {
    case standBy:
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        break;
    case Track:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
        break;
    case Lost:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        break;
    }

    switch(sensorState) // Controls LED2
    {
    case onlyLeft: // LED2 is RED
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
        break;
    case moreLeft: // LED2 is YELLOW
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
        break;
    case Equal: // LED2 is GREEN
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
        break;
    case moreRight: // LED2 is CYAN
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
        break;
    case onlyRight: // LED2 is BLUE
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
        break;
    case None: // LED2 is WHITE
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
        break;
    }

    switch(motorState) //RSLK LEDs
    {
    case Forward:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
        break;
    case Reverse:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);
        break;
    case Stay:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);
        break;
    }
}

/*
 * Name: configPWMTimer
 * Description: Configure PWM Timer
 * Author: Provided by lab
 * Inputs:
 *      clockPeriod, uint16_t, sets period of PWM
 *      clockDivider, uint16_t, sets divider of PWM
 *      duty, uint16_t, sets duty cycle of PWM
 *      channel, uint16_t, sets timer channel
 * Outputs: none
 */
void configPWMTimer(uint16_t clockPeriod, uint16_t clockDivider, uint16_t duty,
                    uint16_t channel)
{
    const uint32_t TIMER=TIMER_A0_BASE;
    uint16_t dutyCycle = duty*clockPeriod/100;
    timerPWMConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    timerPWMConfig.clockSourceDivider = clockDivider;
    timerPWMConfig.timerPeriod = clockPeriod;
    timerPWMConfig.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;
    timerPWMConfig.compareRegister = channel;
    timerPWMConfig.dutyCycle = dutyCycle;
    MAP_Timer_A_generatePWM(TIMER, &timerPWMConfig);
    MAP_Timer_A_stopTimer(TIMER);
}

/*
 * Name: configMotors
 * Description: Configure motors
 * Author: Kennyth Kouch
 * Inputs: none
 * Outputs: none
 */
void configMotors()
{
    if(robotState != standBy)
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); //L Sleep
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); // R Sleep
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4); // L-Wheel Direction
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5); // R-Wheel Direction
        motorState = Forward;
        switch(sensorState)
        {
        case onlyLeft: //LED2 RED
            leftPeriod = 15;
            rightPeriod = 35;
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4); // L-Wheel Direction
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5); // R-Wheel Direction
            break;
        case moreLeft: // LED2 YELLOW
            leftPeriod = 25;
            rightPeriod = 35;
            break;
        case Equal: // LED2 GREEN
            leftPeriod = 35;
            rightPeriod = 35;
            break;
        case moreRight: //LED2 CYAN
            leftPeriod = 35;
            rightPeriod = 25;
            break;
        case onlyRight: //LED2 BLUE
            leftPeriod = 35;
            rightPeriod = 15;
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4); // L-Wheel Direction
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5); // R-Wheel Direction
            break;
        case None: //LED2 WHITE
            robotState = Lost;
            motorState = Reverse;
            leftPeriod = 30;
            rightPeriod = 35;
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4); // L-Wheel Direction
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5); // R-Wheel Direction
            break;
        }
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); //L Sleep
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // R Sleep
    }

    configPWMTimer(leftPeriod,leftDivider,leftDuty, LEFTCHANNEL);
    configPWMTimer(rightPeriod,rightDivider,rightDuty, RIGHTCHANNEL);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}
