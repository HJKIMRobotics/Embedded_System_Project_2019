#include "mbed.h"
#include "C12832.h"
#include "QEI.h"
#include <math.h>

/*-------------------------------Hardware Constants------------------------------*/
#define pi 3.14159f
#define DIAMETER_OF_WHEELS 0.081f //in meters
#define PERIMETER_OF_WHEELS 0.2545f //0.25446900494077325231547411404564 meters
#define RADIUS_OF_ROTATIONR 0.223f //in meters
/*---------------------------Hardware Constants ends----------------------------*/

/*----------------------------Constants for Slight Adjustment--------------------------*/
#define DISTANCE_OF_TURNING_90_DEG 0.3503f //0.35028758087526194608858473723566 meters
#define DISTANCE_OF_TURNING_180_DEG 0.7006f //meters
#define DELTA_TIME_FOR_SPEED_CALCULATION 0.1f //0.000012 second which is the minimum time interval for ticker
#define ADDITIONAL_DISTANCE_OF_U_TURN 0.03f //depend on the drag of the ground, the closet one was 0.03
#define INITIAL_LEFT_WHEEL_PWM_DUTY_CYCLE 0.3f//0.3f
#define INITIAL_RIGHT_WHEEL_PWM_DUTY_CYCLE 0.31f//0.31f
#define U_TURN_WHEELS_PWM_DUTY_CYCLE 0.25f
#define MOTORS_PWM_PERIOD 0.00067f // frequency: 1492.51Hz
#define SENSOR_BLACK_LIMIT 0.5f
#define SENSOR_BLACK_LIMIT_FOR_SENSOR_L2 0.3f
/*------------------------Constants for Slight Adjustment ends-------------------------*/

/*------------------------Control Constants-------------------------*/
#define INITIAL_DUTY_CYCLE 1.0f //runing at 60% duty cycle of PMW
#define KP 0.76f //initial value
#define KI 0.0f //initial value
#define KD 0.0125f //initial value
#define PID_CALCULATION_TIME_DURATION 0.01f //also, this is the time interval for ticker
#define PID_INTEGRAL_UPPER_LIMIT 0.5f 
#define PID_INTEGRAL_LOWER_LIMIT -0.5f 

/*KP = 0.5 is good for initial 0.7(30% duty ratio)*/
/*KP = 0.7, initial 0.5(50% duty ratio), maybe bad*/
/*KP = 0.65, initial 0.4(60% duty ratio), maybe well on a rough ground*/
#define KP_OF_MOTOR_SPEED 0.025f
#define KI_OF_MOTOR_SPEED 0
#define KD_OF_MOTOR_SPEED 0
//#define PID_CONSTANT_SPEED_CALCULATION_TIME_DURATION 0.015f // this is combined in PID
#define DUTY_CYCLE_OF_PWM_TO_SPEED_RATIO 1.71f //was 2.215
#define SPEED_PID_INTEGRAL_UPPER_LIMIT 1.5f
#define SPEED_PID_INTEGRAL_LOWER_LIMIT -1.5f
/*------------------------Control Constants ends--------------------*/

/*------------------------Time Constant-----------------------------*/
#define BLUETOOTH_SIGNAL_READING_TIME_INTERVAL 0.01f
#define ADDITIONAL_RUNING_DISTANCE_WHEN_NOTHING_DETECTED 0.002f //was 0.2, was 0.5
/*------------------------Time Constant ends------------------------*/


/*-----------------Pins Definition---------------*/
#define QUADRATURE_ENCODER_LEFT_MOTOR_CHA_PIN D4 //PWM3/1, also SPI1_MOSI
#define QUADRATURE_ENCODER_LEFT_MOTOR_CHB_PIN D5 //also SPI1_MOSI, D5 is used by red LED, maybe need to change to PC4
#define QUADRATURE_ENCODER_RIGHT_MOTOR_CHA_PIN D2 //UART1_RX
#define QUADRATURE_ENCODER_RIGHT_MOTOR_CHB_PIN D3 //UART2_TX

#define POTENTIOMETER_LEFT_PIN A0
#define POTENTIOMETER_RIGHT_PIN A1

#define SENSOR_CENTER_L_PIN PC_2 // new: was PC_1, was PC_2, was PA_4/A2, new pin, need to test try PC_2, tried PC1, doesn't read
#define SENSOR_CENTER_R_PIN PB_1  //NOT working, always on ->r1
#define SENSOR_LEFT_1_PIN PC_5  //working ->r3 
#define SENSOR_LEFT_2_PIN PC_0 // is A5-right joystick, WAS using PA_4
#define SENSOR_RIGHT_1_PIN PC_4 //working ->r2
#define SENSOR_RIGHT_2_PIN PC_3  // also SPI2_MOSI, working well WHY PC_1 IS CONNECTING? was PC_2???

#define MOTOR_LEFT_PWM_PIN PB_0 // same pin as: PWM1/2N, down joystick
#define MOTOR_RIGHT_PWM_PIN PC_8    // same pin as: PWM3/3

#define MOTOR_LEFT_DIRECTION_PIN PA_14
#define MOTOR_RIGHT_DIRECTION_PIN  PC_12

#define MOTOR_LEFT_BIPOLAR_PIN PA_13
#define MOTOR_RIGHT_BIPOLAR_PIN PC_10

#define MOTOR_ENABLE_PIN PB_7

#define BLUETOOTH_RX_PIN PA_12  //UART6_RX, UART1_RTS this connect to TXD
#define BLUETOOTH_TX_PIN PA_11 //UART6_TX also USART1_CTS this connect to RXD
/*-------------Pins Definition ends--------------*/

C12832 Debug(D11, D13, D12, D7, D10);// same pins: PA_7, PA_5, PA_6, PA_8, PB_6
                                     //BuggyTimer counts the time in deciseconds since the buggy is on the white line
class BuggyTimer {
public:
    BuggyTimer(); //initialise on time
    void countTimeInMillisecond();
    void timeTic();
    int getOnTimeInMillisecond();
    
protected:
    Ticker ticker1Timer;

private:
    int onTimeInMillisecond; //how long the buggy is on
};


class QuadratureEncoder: public QEI {
public:
    QuadratureEncoder(PinName lcha, PinName lchb, PinName lnc, int lren, QEI::Encoding lxn);
    float mileageInMeters(); //realy in meter?
    void calculateInstantaneousSpeed();
    float getInstantaneousSpeed();
    float getAccurateInstantaneousSpeed();

    /*QuadratureEncoder*/
    float getFractionalRevolutions(); //

	/*For race: measuring the track*/

private:
    float instantaneousSpeed;
    float mileageBeforeDeltaT;
    
    /*For calculate average speed on 3 readings*/
    float lastInstantaneousSpeed;
    float previousInstantaneousSpeed;
    float previousLastInstantaneousSpeed;
    float accurateInstantaneousSpeed;

	/*For race: measuring the track*/
	float temMileageBeforeNegativeDirection;
	float negativeMileage;
};

class LCDDisplay : public C12832 {
public:
    LCDDisplay();
    void display(int x, int y, char words[50]);
private:
};

class Joystick {
private:
	DigitalIn leftJoystick, rightJoystick, centerJoystick, upJoystick, downJoystick;
public:
	Joystick(PinName lj, PinName rj, PinName cj, PinName uj, PinName dj)
		: leftJoystick(lj), rightJoystick(rj), centerJoystick(cj), upJoystick(uj), downJoystick(dj) {}

	bool leftJoyPushed() {           //returns joystick's status
		return leftJoystick;
	}
	bool rightJoyPushed() {
		return rightJoystick;
	}
	bool centerJoyPushed() {
		return centerJoystick;
	}
	bool upJoyPushed() {
		return upJoystick;
	}
	bool downJoyPushed() {
		return downJoystick;
	}
};

class Potentiometer {
public:
    Potentiometer();
    float getLeftPotRatio();
    float getRightPotRatio();
private:
    AnalogIn leftPot, rightPot;
};


class Sensor {
public:
    Sensor(PinName scl, PinName sc0, PinName sl1, PinName sl2, PinName sr1, PinName sr2);
    bool positionCenterL();
    bool positionCenterR();
    bool positionLeft1();
    bool positionLeft2();
    bool positionRight1();
    bool positionRight2();
    float getsensorInputCenterLPercentage();
    float getsensorInputCenterRPercentage();
    float getSensorInputLeft1Percentage();
    float getSensorInputLeft2Percentage();
    float getSensorInputRight1Percentage();
    float getSensorInputRight2Percentage();
	
	//Debug function:
	void showSensorState();// while(1){} inside
private:
    AnalogIn sensorInputCenterL, sensorInputCenterR, sensorInputLeft1, sensorInputLeft2, sensorInputRight1, sensorInputRight2;
};

class Motor : public QuadratureEncoder{
public:
    Motor(/*QuadratureEncoder pins*/PinName lcha, PinName lchb, PinName lnc, int lren, QEI::Encoding lxn
		/*Motor pins*/, PinName mlp, PinName mld, PinName mlb, PinName me);
	
	void setMotorPWMPeriod(float lp);
    void motorInitialSettings();
    void goingForward(void);
    float getPWMDutyCycle();
    void motorBreak();
    void setMotorDirection(bool direction);
	bool getMotorDirection();
	float fixPWMDutyCycleByMultiplyLRSpeedRatio(float dutyCycle);
    void setPWMDutyCycle(float dutyCycle);
    float getTheoreticalSpeed(float dutyCycle);
	void setLRMotorsSpeedRatio(float hDutyCycleSpeedConvRate, float lDutyCycleSpeedConvRate);

private:
    PwmOut motorPWM;
    DigitalOut motorDirection;
    DigitalOut motorBipolar;
    DigitalOut motorEnable;
	float lRMotorHighSpeedRatioForSamePWMDutyCycle;
	float lRMotorLowSpeedRatioForSamePWMDutyCycle;
};

class BluetoothModule : public Serial {
public:
    BluetoothModule(PinName TXD, PinName RXD);
private:
};

class PID {
public:
private:
};

class Navigation : protected BuggyTimer {
public:
    Navigation(Motor* dbp, Motor* dbpr, BluetoothModule* btprt, Sensor* sprt, Potentiometer* pptr, Joystick* jskp);
    void doUturn();
    void actionsWhenRecivingBluetoothSignal();
	void buggyInitialisation();
	
	//Refresh Instantaneous Speed By Ticker:
	void calculateInstantaneousSpeedForBothWheels();
	void refreshInstantaneousSpeedByTicker();
	void detachRefreshInstantaneousSpeedByTicker();

    //Control functions - PID control:
    void calculateErrorVariables();
    float calculateControlledLeftDutyCycleByUsingPID();
    float calculateControlledRightDutyCycleByUsingPID();
    void writePWMDutyCycleFromPIDToBothWheel();
    void refreshDutyCycleForBothWheel(); // ticker inside this function
    void detachRefreshDutyCycleForBothWheel();
    float calculateControlledLeftPWMByUsingPID(float PWMFromTurningPID);
    float calculateControlledRightPWMByUsingPID(float PWMFromTurningPID);
	bool isPIDSpeedControlOn;
	void setKPID(float kp, float ki, float kd);
	void setInitialPWMDutyCycle(float IPD);
	void changePWMDutyCycleRunOnSlope();

	//Adjustment functions for hardware:
	void balanceMotorDutyCycleToSpeedConversionRate();

	//Race Functions:
	void showActualMileage();
	void calculateActualMileage();

	//Debug functions:
	void setSpeedByUsingPotentiometerAndDisplay(); //while(1){} loop inside
	void showSpeedAndPWMDutyCycle(); 
	float getPositionError();
	void setKPIDByUsingPotAndJoy();
private:
	Motor* LeftMotorPtr;
	Motor* RightMotorPtr;
    BluetoothModule* BluetoothModulePtr;
    Sensor* SensorPtr;
	Potentiometer* PotentiometerPtr;
	Joystick* JoystickPtr;

	//Bluetooth:
    char textRecivedFromBlutooth; //text Recived From Blutooth device (like phone)
    
	Ticker ticker2ForRefreshPWMDutyCycle;

    //Control variables:
	float originalInitialDutyCycle; //potentiometer can set this, original setting of initial Duty Cycle, it shouldn't change after initialisation/potentiometer's seting
	float Kp, Ki, Kd, initialDutyCycle;
    float positionError, lastPositionError, positionErrorDerivative, positionErrorIntegral, lastPositionErrorIntegral;
    float temMileageRecordForPID;
    bool motorStopFlag;
	bool isLeftTheEdgeSensorReached;
	bool isRightTheEdgeSensorReached;
	bool isSpeedControlOn;
    float leftSpeedError, leftLastSpeedError, leftSpeedErrorDerivative, leftSpeedErrorIntegral, leftLastSpeedErrorIntegral;
    float rightSpeedError, rightLastSpeedError, rightSpeedErrorDerivative, rightSpeedErrorIntegral, rightLastSpeedErrorIntegral;
    float leftPWMControlValue, lastLeftPWMControlValue;
    float rightPWMControlValue, lastRightPWMControlValue;
	float leftOverFlowedPWMDutyCycle;// for right wheel 
	float rightOverFlowedPWMDutyCycle;// for left wheel

	//For Refresh Speed 
	Ticker deltaTTimer;

	//Variables for race
	float actualMileage;
};
