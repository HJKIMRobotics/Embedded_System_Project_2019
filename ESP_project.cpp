/*---------------------------------What's new in this version:------------------------------*/
/*Final code for race																		*/
/*Function not done: speed decreasing when going to road curve				                */
/*New Bluetooth functions for speed changing are included						            */
/*Code optimisation and rearrange not finished											*/			
/*------------------------------------------------------------------------------------------*/

#include "main.h"

int main() {
    LCDDisplay* _LCDDisplay = new LCDDisplay();
    BuggyTimer* _BuggyTimer = new BuggyTimer();
	Potentiometer* _Potentiometer = new Potentiometer();
	Joystick* _Joystick = new Joystick(A4, NC, D4, A2, NC);
	Motor* _LeftMotor = new Motor(QUADRATURE_ENCODER_LEFT_MOTOR_CHA_PIN, QUADRATURE_ENCODER_LEFT_MOTOR_CHB_PIN, NC, 256, QEI::X4_ENCODING,
		MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIRECTION_PIN, MOTOR_LEFT_BIPOLAR_PIN, MOTOR_ENABLE_PIN);
	Motor* _RightMotor = new Motor(QUADRATURE_ENCODER_RIGHT_MOTOR_CHB_PIN, QUADRATURE_ENCODER_RIGHT_MOTOR_CHA_PIN, NC, 256, QEI::X4_ENCODING,
		MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIRECTION_PIN, MOTOR_RIGHT_BIPOLAR_PIN, NC);

	//now QuadratureEncoder/Dashboard class is in Motor （left and right）should I write this in the class or main?
    BluetoothModule* _BluetoothModule = new BluetoothModule(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
    Sensor* _Sensor = new Sensor(SENSOR_CENTER_L_PIN, SENSOR_CENTER_R_PIN, SENSOR_LEFT_1_PIN, SENSOR_LEFT_2_PIN, SENSOR_RIGHT_1_PIN, SENSOR_RIGHT_2_PIN);
    Navigation* _Navigation = new Navigation(_LeftMotor, _RightMotor, _BluetoothModule, _Sensor, _Potentiometer, _Joystick);
    //Timer timerForPID;

    _BluetoothModule->baud(9600);

	//_Sensor->showSensorState();//debug: sensor returned value

    Debug.locate(0, 0);
    Debug.printf("Starting, wait for 1s");
    wait(0.2);
    Debug.cls();
	_Navigation->setKPIDByUsingPotAndJoy(); //debug: tune PID control variables
    //_BuggyTimer->timeTic();
    _Navigation->buggyInitialisation();
    
    // initialise ticker for Detecting Speed of Both Wheel:
	_Navigation->refreshInstantaneousSpeedByTicker();
    /*Ticker inside this function. It will be always on. Hence, don't need to detach the function for this ticker. No reason to put its ticker to 'main'
    printf woundn't work if we use it in the ticker, because printf takes too soon to run?
    it seems the ticker can't be too short. if it's too short, it affect the code in while loop
    */
    //float deltaTOfPIDController = 0; 

	_Navigation->balanceMotorDutyCycleToSpeedConversionRate(); // blance Motor Duty Cycle To Speed Conver sionRate(left motor is higher) for both wheels

	/*----Debug: change dutyCycle by using potentiometer to check speed/dutyCycle relationship---*/
	//_Navigation->setSpeedByUsingPotentiometerAndDisplay(); //testing speed, pwm relationship
	/*------------------------------------------------------------------------------------------ */

    _Navigation->refreshDutyCycleForBothWheel(); //ticker inside

    while (1) {
		//_Navigation->showSpeedAndPWMDutyCycle();//debug: show speed and pwm duty cycle
        _Navigation->actionsWhenRecivingBluetoothSignal();

        // PID contorller:
        //timerForPID.start();
        //_Navigation->writePWMDutyCycleFromPIDToBothWheel();
        /*timerForPID.stop();
        deltaTOfPIDController = timerForPID.read();
        timerForPID.reset();*/

        /*Debug.locate(0, 0);
        Debug.printf("%.2f %.2f %.2f %.2f %.2f %.2f", _Sensor->getSensorInputLeft2Percentage(), _Sensor->getSensorInputLeft1Percentage(), _Sensor->getsensorInputCenterLPercentage(), _Sensor->getsensorInputCenterRPercentage(), _Sensor->getSensorInputRight1Percentage(), _Sensor->getSensorInputRight2Percentage());
        Debug.locate(0, 10);
        Debug.printf("L2:%d L1:%d CL:%d CR:%d R1:%d R2%d", _Sensor->positionLeft2(), _Sensor->positionLeft1(), _Sensor->positionCenterL(), _Sensor->positionCenterR(), _Sensor->positionRight1(), _Sensor->positionRight2());
        Debug.locate(0, 20);
        Debug.printf(" LP:%2f RP:%2f", _Navigation->getPWMDutyCycle(), _Navigation->getRightPWM());*/
        
        /*Debug.locate(0, 0);
        Debug.printf("L Sp:%.2f", _LeftMotor->getInstantaneousSpeed());
        Debug.locate(60, 0);
        Debug.printf("R Sp:%.2f", _RightMotor->getInstantaneousSpeed());
        Debug.locate(0, 15);
        Debug.printf("L mileage:%.2f", _LeftMotor->mileageInMeters());
        Debug.locate(65, 15);
        Debug.printf("R mileage:%.2f", _RightMotor->mileageInMeters());*/
    }
}
/*********************BuggyTimer***************************/
BuggyTimer::BuggyTimer() : onTimeInMillisecond(0) {}
void BuggyTimer::countTimeInMillisecond() {
    ++onTimeInMillisecond;
}
void BuggyTimer::timeTic() {
    ticker1Timer.attach(callback(this, &BuggyTimer::countTimeInMillisecond), 0.1);
}
int BuggyTimer::getOnTimeInMillisecond() { return onTimeInMillisecond; }


/**************************QuadratureEncoder*************************/
QuadratureEncoder::QuadratureEncoder(PinName lcha, PinName lchb, PinName lnc, int lren, QEI::Encoding lxn):QEI(lcha, lchb, lnc, lren, lxn) {
	instantaneousSpeed = 0;
	lastInstantaneousSpeed = previousLastInstantaneousSpeed = 0;
    mileageBeforeDeltaT = 0;
	temMileageBeforeNegativeDirection = negativeMileage = 0;
}
// motor right channel A D2, channel B D3. pulse and revolution counts in negative if use channel A D2, channel B D3, so reverse to CHA-D3 CHB-D2

float QuadratureEncoder::mileageInMeters() {
    return getFractionalRevolutions() * PERIMETER_OF_WHEELS;
}

void QuadratureEncoder::calculateInstantaneousSpeed() {
    instantaneousSpeed = (mileageInMeters() - mileageBeforeDeltaT) / DELTA_TIME_FOR_SPEED_CALCULATION;
    mileageBeforeDeltaT = mileageInMeters(); //reset distance 1
}

float QuadratureEncoder::getInstantaneousSpeed() {
    return instantaneousSpeed;
}

float QuadratureEncoder::getAccurateInstantaneousSpeed(){
    if (previousLastInstantaneousSpeed == 0)
        previousLastInstantaneousSpeed = getInstantaneousSpeed();
    if (lastInstantaneousSpeed == 0)
        lastInstantaneousSpeed = getInstantaneousSpeed();
    accurateInstantaneousSpeed = (previousLastInstantaneousSpeed + lastInstantaneousSpeed + getInstantaneousSpeed()) / 3.0f;
    previousLastInstantaneousSpeed = lastInstantaneousSpeed;
    lastInstantaneousSpeed = getInstantaneousSpeed();
    return accurateInstantaneousSpeed;
}

/*QuadratureEncoder*/
float QuadratureEncoder::getFractionalRevolutions() {  //return how many rotations left wheel rotated. if rotated for 60% of the wheel, it returns 0.6 
    return getPulses() / (4.0f*256.0f);
}


/**************************LCD display**********************/

LCDDisplay::LCDDisplay() : C12832(D11, D13, D12, D7, D10) {}
void LCDDisplay::display(int x, int y, char words[50]) {
    locate(x, y);
    printf(words);
}

/**************************Potentiometer*******************/

Potentiometer::Potentiometer() : leftPot(POTENTIOMETER_LEFT_PIN), rightPot(POTENTIOMETER_RIGHT_PIN) {}
float Potentiometer::getLeftPotRatio() {
    return leftPot;
}
float Potentiometer::getRightPotRatio() {
    return rightPot;
}
/**************************Sensors**********************/
Sensor::Sensor(PinName scl, PinName scr, PinName sl1, PinName sl2, PinName sr1, PinName sr2) :sensorInputCenterL(scl), sensorInputCenterR(scr), sensorInputLeft1(sl1), sensorInputLeft2(sl2), sensorInputRight1(sr1), sensorInputRight2(sr2) {}

bool Sensor::positionCenterL() { 
    if (sensorInputCenterL.read() < SENSOR_BLACK_LIMIT) {
        return 1;
    }
    else {
        return 0;
    }
}
bool Sensor::positionCenterR() { // notice, due to the problem of PCB, the reading of voltage is reversed. When the sensor reads 90%, it actually reads 10%. So change the compare logic to opposite
    if (sensorInputCenterR.read() < SENSOR_BLACK_LIMIT) {
        return 1;
    }
    else {
        return 0;
    }
}
bool Sensor::positionLeft1() {
    if (sensorInputLeft1.read() < SENSOR_BLACK_LIMIT) {
        return 1;
    }
    else {
        return 0;
    }
}
bool Sensor::positionLeft2() {
    if (sensorInputLeft2.read() < SENSOR_BLACK_LIMIT_FOR_SENSOR_L2) {
        return 1;
    }
    else {
        return 0;
    }
}
bool Sensor::positionRight1() {
    if (sensorInputRight1.read() < SENSOR_BLACK_LIMIT) {
        return 1;
    }
    else {
        return 0;
    }
}
bool Sensor::positionRight2() {
    if (sensorInputRight2.read() < SENSOR_BLACK_LIMIT) {
        return 1;
    }
    else {
        return 0;
    }
}

float Sensor::getsensorInputCenterLPercentage() {
    return sensorInputCenterL;
}
float Sensor::getsensorInputCenterRPercentage() {
    return sensorInputCenterR;
}
float Sensor::getSensorInputLeft1Percentage() {
    return sensorInputLeft1;
}
float Sensor::getSensorInputLeft2Percentage() {
    return sensorInputLeft2;
}
float Sensor::getSensorInputRight1Percentage() {
    return sensorInputRight1;
}
float Sensor::getSensorInputRight2Percentage() {
    return sensorInputRight2;
}


//**********************Motor**************************//
Motor::Motor(/*QuadratureEncoder pins*/PinName lcha, PinName lchb, PinName lnc, int lren, QEI::Encoding lxn
	/*Motor pins*/, PinName mlp, PinName mld, PinName mlb, PinName me)
    :QuadratureEncoder(lcha, lchb, lnc, lren, lxn)
	, motorPWM(mlp), motorDirection(mld), motorBipolar(mlb), motorEnable(me) {
	lRMotorHighSpeedRatioForSamePWMDutyCycle = lRMotorLowSpeedRatioForSamePWMDutyCycle = 1.0f; //initialise left / right Motor Speed Ratios For Same PWM Duty Cycle
}

void Motor::setMotorPWMPeriod(float lp) {
	motorPWM.period(lp);
}
void Motor::motorInitialSettings() {
    motorEnable = 1;
    motorDirection = 1; //set left motor going forward
    motorBipolar = 0; //set left motor unipolar
    motorPWM = INITIAL_LEFT_WHEEL_PWM_DUTY_CYCLE;//PWM is not reversed anymore
	setMotorPWMPeriod(MOTORS_PWM_PERIOD);//frequency: 1492.51Hz, WARNING!!!:This has to be after other initial setting(maybe dutyCycle setting?)
}

void Motor::goingForward(void) {
    motorDirection = 1;
    motorPWM = INITIAL_LEFT_WHEEL_PWM_DUTY_CYCLE;
}
float Motor::getPWMDutyCycle() {
    return 1.0f-motorPWM;
}

void Motor::motorBreak() {
    
    //motorPWM is reversed
    if (motorPWM != 1)
		motorDirection = !motorDirection;
    wait(0.0002f); // can't be larger than PID_CALCULATION_TIME_DURATION
    motorPWM = 1;//PWM 1 means motor is stopped(reversed pwm)
    motorDirection = 1;
}

void Motor::setMotorDirection(bool direction) {
    motorDirection = direction;
}

bool Motor::getMotorDirection() {
	return motorDirection;
}

float Motor::fixPWMDutyCycleByMultiplyLRSpeedRatio(float dutyCycle){
	//set duty cycle limit, is it necessary?
	if (dutyCycle*lRMotorHighSpeedRatioForSamePWMDutyCycle > 1.0f)
		return 1.0f;
	else if (dutyCycle < 0.0f)
		return 0.0f;

	else if (dutyCycle <= 0.3f)
		return dutyCycle*lRMotorLowSpeedRatioForSamePWMDutyCycle;
	else
		return dutyCycle*lRMotorHighSpeedRatioForSamePWMDutyCycle;
}

void Motor::setPWMDutyCycle(float dutyCycle) {
    motorPWM = 1.0f - fixPWMDutyCycleByMultiplyLRSpeedRatio(dutyCycle);
}

float Motor::getTheoreticalSpeed(float dutyCycle) {
    return DUTY_CYCLE_OF_PWM_TO_SPEED_RATIO*dutyCycle;
}

void Motor::setLRMotorsSpeedRatio(float hDutyCycleSpeedConvRate, float lDutyCycleSpeedConvRate){
	lRMotorHighSpeedRatioForSamePWMDutyCycle = hDutyCycleSpeedConvRate;
	lRMotorLowSpeedRatioForSamePWMDutyCycle = lDutyCycleSpeedConvRate;
}


//********************Bluetooth Module********************//
BluetoothModule::BluetoothModule(PinName TXD, PinName RXD) :Serial(TXD, RXD) {}


//********************Navigation**********************//
Navigation::Navigation(Motor* dbp, Motor* dbpr, BluetoothModule* btprt, Sensor* sprt, Potentiometer* pptr, Joystick* jskp) : LeftMotorPtr(dbp), RightMotorPtr(dbpr), BluetoothModulePtr(btprt), SensorPtr(sprt), PotentiometerPtr(pptr), JoystickPtr(jskp){
	LeftMotorPtr->setPWMDutyCycle(0);
	RightMotorPtr->setPWMDutyCycle(0);
	originalInitialDutyCycle = INITIAL_DUTY_CYCLE;
	Kp = KP;
	Ki = KI;
	Kd = KD;
	initialDutyCycle = INITIAL_DUTY_CYCLE;
    positionError = lastPositionError = positionErrorDerivative = positionErrorIntegral = lastPositionErrorIntegral = 0; //initialise PID variables 
    temMileageRecordForPID = 0;
    motorStopFlag = 0;
	isLeftTheEdgeSensorReached = isRightTheEdgeSensorReached = 0;
    leftPWMControlValue = rightPWMControlValue = lastLeftPWMControlValue = lastRightPWMControlValue = 0;
    leftSpeedError = 0.0001f; // to avoid /0 error
    leftLastSpeedError = leftSpeedErrorDerivative = leftSpeedErrorIntegral = leftLastSpeedErrorIntegral = 0;
    rightSpeedError = 0.0001f; // to avoid /0 error
    rightLastSpeedError = rightSpeedErrorDerivative = rightSpeedErrorIntegral = rightLastSpeedErrorIntegral = 0;
	leftOverFlowedPWMDutyCycle = rightOverFlowedPWMDutyCycle = 0;
	isPIDSpeedControlOn = 0;//
	isSpeedControlOn = 0;
	//Variables for race
	actualMileage = 0;

}

void Navigation::doUturn() {
	LeftMotorPtr->setMotorDirection(0);
	RightMotorPtr->setMotorDirection(1);
	LeftMotorPtr->setPWMDutyCycle(U_TURN_WHEELS_PWM_DUTY_CYCLE + 0.2f);
	RightMotorPtr->setPWMDutyCycle(U_TURN_WHEELS_PWM_DUTY_CYCLE);
    wait(0.3);

    while (true) {
        // left wheel is runing in reversed direction, so the mileageInMeters is negative
        if (SensorPtr->positionLeft2()||SensorPtr->positionLeft1()|| SensorPtr->positionCenterL()|| SensorPtr->positionCenterR()|| SensorPtr->positionRight1())
            break;
    }
    /*Debug.cls();
    Debug.locate(0, 0);
    Debug.printf("current PWM:%.2f", getPWMDutyCycle());
    Debug.locate(0, 15);
    Debug.printf(" difference:%.2f", LeftMotorPtr->mileageInMeters() - currentmileageInMeters);
    */
	LeftMotorPtr->setMotorDirection(1);
}

void Navigation::actionsWhenRecivingBluetoothSignal() {
    if (BluetoothModulePtr->readable()) {  //will use this function in main
    textRecivedFromBlutooth = BluetoothModulePtr->getc();
	if (textRecivedFromBlutooth == 't') { //turning mode
		setKPID(0.72f, 0.0f, 0.015f);
		setInitialPWMDutyCycle(0.7f);
		isSpeedControlOn = 1;
	}
	else if (textRecivedFromBlutooth == 's') { //straight line mode
		setKPID(0.36f, 0.0f, 0.015f);
		setInitialPWMDutyCycle(1.0f);
		isSpeedControlOn = 0;
	}
	else if (textRecivedFromBlutooth == 'd') { //down slope mode
		setKPID(0.72f, 0.0f, 0.01f);
		setInitialPWMDutyCycle(0.6f);
		isSpeedControlOn = 0;
	}
	else if (textRecivedFromBlutooth == 'a') { //auto mode
		setKPID(0.72f, 0.0f, 0.015f);
		setInitialPWMDutyCycle(0.8f);
		isSpeedControlOn = 1;
	}
	else if (textRecivedFromBlutooth == 'h') { //high speed mode
		setKPID(0.72f, 0.0f, 0.015f);
		setInitialPWMDutyCycle(1.0f);
		isSpeedControlOn = 0;
	}
    else if (textRecivedFromBlutooth == 'U') {
        detachRefreshDutyCycleForBothWheel();
        doUturn();
        refreshDutyCycleForBothWheel();
    }
    else if (textRecivedFromBlutooth == 'H') {
        Debug.locate(0, 0);
        Debug.printf("Hello World!");
    }
	else if (textRecivedFromBlutooth == 'S') {
		showActualMileage();
	}
    }
}

void Navigation::buggyInitialisation() {
	//motorInitialSettings includes: motorEnable，motorDirection，motorBipolar，motorPWM
	LeftMotorPtr->motorInitialSettings();
	RightMotorPtr->motorInitialSettings();
	// reset encoders
	LeftMotorPtr->reset();
	RightMotorPtr->reset();
}

void Navigation::calculateInstantaneousSpeedForBothWheels() {
	LeftMotorPtr->calculateInstantaneousSpeed();
	RightMotorPtr->calculateInstantaneousSpeed();
	if (isSpeedControlOn == 1) {
		changePWMDutyCycleRunOnSlope();//speed control
	}
}

void Navigation::refreshInstantaneousSpeedByTicker() {
	deltaTTimer.attach(callback(this, &Navigation::calculateInstantaneousSpeedForBothWheels), DELTA_TIME_FOR_SPEED_CALCULATION);
}

void Navigation::detachRefreshInstantaneousSpeedByTicker() {
	deltaTTimer.detach();
}

//This function takes the position values from sensors and calculates the error in position. The reference value is when the centre sensor is white and the others are black.
void Navigation::calculateErrorVariables() {

    if (SensorPtr->positionLeft2() == 0 && SensorPtr->positionLeft1() ==0 && SensorPtr->positionCenterL() ==1 && SensorPtr->positionCenterR() == 1 && SensorPtr->positionRight1() == 0 && SensorPtr->positionRight2() ==0 ) {
        positionError = 0;
        motorStopFlag = 0;
		isLeftTheEdgeSensorReached = isRightTheEdgeSensorReached = 0;
        //001100
    }

    else if (SensorPtr->positionLeft2() == 0 && SensorPtr->positionLeft1() == 0 && SensorPtr->positionCenterL() == 1 && SensorPtr->positionCenterR() == 0 && SensorPtr->positionRight1() == 0 && SensorPtr->positionRight2() == 0) {
        positionError = 0.05f;
        motorStopFlag = 0;
		isLeftTheEdgeSensorReached = isRightTheEdgeSensorReached = 0;
        //001000
    }

    else if (SensorPtr->positionLeft2() == 0 && SensorPtr->positionLeft1() == 0 && SensorPtr->positionCenterL() == 0 && SensorPtr->positionCenterR() == 1 && SensorPtr->positionRight1() == 0 && SensorPtr->positionRight2() == 0) {
        positionError = 0.05f;
        motorStopFlag = 0;
		isLeftTheEdgeSensorReached = isRightTheEdgeSensorReached = 0;
        //000100
    }

    else if (SensorPtr->positionLeft2() == 0 && SensorPtr->positionLeft1() == 1 && SensorPtr->positionCenterL() == 1 && SensorPtr->positionCenterR() == 0 && SensorPtr->positionRight1() == 0 && SensorPtr->positionRight2() == 0) {
        positionError = 0.2f;
        motorStopFlag = 0;
		isLeftTheEdgeSensorReached = isRightTheEdgeSensorReached = 0;
        //011000
    }

    else if (SensorPtr->positionLeft2() == 0 && SensorPtr->positionLeft1() == 0 && SensorPtr->positionCenterL() == 0 && SensorPtr->positionCenterR() == 1 && SensorPtr->positionRight1() == 1 && SensorPtr->positionRight2() == 0) {
        positionError = -0.2f;
        motorStopFlag = 0;
		isLeftTheEdgeSensorReached = isRightTheEdgeSensorReached = 0;
        //000110
    }

    else if (SensorPtr->positionLeft2() == 0 && SensorPtr->positionLeft1() == 1 && SensorPtr->positionCenterL() == 0 && SensorPtr->positionCenterR() == 0 && SensorPtr->positionRight1() == 0 && SensorPtr->positionRight2() == 0) {
        positionError = 0.4f;
        motorStopFlag = 0;
		isLeftTheEdgeSensorReached = isRightTheEdgeSensorReached = 0;
        //010000
    }

    else if (SensorPtr->positionLeft2() == 0 && SensorPtr->positionLeft1() == 0 && SensorPtr->positionCenterL() == 0 && SensorPtr->positionCenterR() == 0 && SensorPtr->positionRight1() == 1 && SensorPtr->positionRight2() == 0) {
        positionError = -0.4f;
        motorStopFlag = 0;
		isLeftTheEdgeSensorReached = isRightTheEdgeSensorReached = 0;
        //000010
    }

    else if (SensorPtr->positionLeft2() == 1 && SensorPtr->positionLeft1() == 1 && SensorPtr->positionCenterL() == 0 && SensorPtr->positionCenterR() == 0 && SensorPtr->positionRight1() == 0 && SensorPtr->positionRight2() == 0) {
        positionError = 0.6f;
        motorStopFlag = 0;
		isLeftTheEdgeSensorReached = isRightTheEdgeSensorReached = 0;
        //110000
    }

    else if (SensorPtr->positionLeft2() == 0 && SensorPtr->positionLeft1() == 0 && SensorPtr->positionCenterL() == 0 && SensorPtr->positionCenterR() == 0 && SensorPtr->positionRight1() == 1 && SensorPtr->positionRight2() == 1) {
        positionError = -0.6f;
        motorStopFlag = 0;
		isLeftTheEdgeSensorReached = isRightTheEdgeSensorReached = 0;
        //000011
    }

    else if (SensorPtr->positionLeft2() == 1 && SensorPtr->positionLeft1() == 0 && SensorPtr->positionCenterL() == 0 && SensorPtr->positionCenterR() == 0 && SensorPtr->positionRight1() == 0 && SensorPtr->positionRight2() == 0) {
        positionError = 0.8f;
        motorStopFlag = 0;
		isLeftTheEdgeSensorReached = 1;
        //100000
    }

    else if (SensorPtr->positionLeft2() == 0 && SensorPtr->positionLeft1() == 0 && SensorPtr->positionCenterL() == 0 && SensorPtr->positionCenterR() == 0 && SensorPtr->positionRight1() == 0 && SensorPtr->positionRight2() == 1) {
        positionError = -0.8f;
        motorStopFlag = 0;
		isRightTheEdgeSensorReached = 1;// it will be 1 if the buggy is out of white line
        //000001
    }

    else if (SensorPtr->positionLeft2() == 0 && SensorPtr->positionLeft1() == 0 && SensorPtr->positionCenterL() == 0 && SensorPtr->positionCenterR() == 0 && SensorPtr->positionRight1() == 0 && SensorPtr->positionRight2() == 0 && isLeftTheEdgeSensorReached == 0 && isRightTheEdgeSensorReached == 0) {
        //000000 when the sensors can't detect anything for 5 cm, stop
        if (temMileageRecordForPID == 0) { // test if the buggy run 5 cm without any detection
            temMileageRecordForPID = (LeftMotorPtr->mileageInMeters() + RightMotorPtr->mileageInMeters()) / 2.0f; //average distance the buggy went
        }
        if ((LeftMotorPtr->mileageInMeters() + RightMotorPtr->mileageInMeters()) / 2.0f - temMileageRecordForPID >= ADDITIONAL_RUNING_DISTANCE_WHEN_NOTHING_DETECTED) {
            motorStopFlag = 1;
            /*Debug.locate(0, 0);
            Debug.printf("TPID: %.2f", temMileageRecordForPID);*/
            temMileageRecordForPID = (LeftMotorPtr->mileageInMeters() + RightMotorPtr->mileageInMeters()) / 2.0f;
        }
        /*Debug.locate(20, 15);
        Debug.printf("TPID: %.2f", (LeftMotorPtr->mileageInMeters() + RightMotorPtr->mileageInMeters()) / 2.0f);*/
		
    }

    if (SensorPtr->positionLeft2() == 0 && SensorPtr->positionLeft1() == 0 && SensorPtr->positionCenterL() == 0 && SensorPtr->positionCenterR() == 0 && SensorPtr->positionRight1() == 0 && SensorPtr->positionRight2() == 0 && isLeftTheEdgeSensorReached == 1 ) {
			positionError = 1.0f;
        //000000 when the buggy ran out of white line. Assume it is on the left possition, becasue it was on the left side of the white line.
    }

    if (SensorPtr->positionLeft2() == 0 && SensorPtr->positionLeft1() == 0 && SensorPtr->positionCenterL() == 0 && SensorPtr->positionCenterR() == 0 && SensorPtr->positionRight1() == 0 && SensorPtr->positionRight2() == 0  && isRightTheEdgeSensorReached == 1) {
			positionError = -1.0f;
        //000000 when the buggy ran out of white line. Assume it is on the right possition, becasue it was on the right side of the white line.
    }

    //calculate error derivative:
	positionErrorDerivative = (positionError - lastPositionError) / PID_CALCULATION_TIME_DURATION;

    //calculate error integral:
    positionErrorIntegral = lastPositionErrorIntegral + positionError*PID_CALCULATION_TIME_DURATION; //PID_CALCULATION_TIME_DURATION is the sampling time
	if (positionErrorIntegral > PID_INTEGRAL_UPPER_LIMIT)
		positionErrorIntegral = PID_INTEGRAL_UPPER_LIMIT;
	if (positionErrorIntegral < PID_INTEGRAL_LOWER_LIMIT)
		positionErrorIntegral = PID_INTEGRAL_LOWER_LIMIT;

    /*refresh variables*/
    lastPositionError = positionError;
    lastPositionErrorIntegral = positionErrorIntegral;
}

float Navigation::calculateControlledLeftDutyCycleByUsingPID() { //constant speed included
    float controlledLeftDutyCycle = (initialDutyCycle - Kp*positionError - Ki*positionErrorIntegral - Kd*positionErrorDerivative) - rightOverFlowedPWMDutyCycle;
    float fixedLeftDutyRaitoForConstantSpeed = controlledLeftDutyCycle + isPIDSpeedControlOn*calculateControlledLeftPWMByUsingPID(controlledLeftDutyCycle); //isPIDSpeedControlOn: speed control switch

    /*Debug.locate(0, 20);
    Debug.printf("Lcs:%.2f", fixedLeftDutyRaitoForConstantSpeed);*/
    
	if (fixedLeftDutyRaitoForConstantSpeed > 1.0f) {
		leftOverFlowedPWMDutyCycle = fixedLeftDutyRaitoForConstantSpeed - 1.0f;
		return 1.0f;
	}
	else if (fixedLeftDutyRaitoForConstantSpeed < -1.0f) {
		leftOverFlowedPWMDutyCycle = fixedLeftDutyRaitoForConstantSpeed + 1.0f;
		return -1.0f;
	}
	else {
		leftOverFlowedPWMDutyCycle = 0; //reset overFlowedPWMDutyCycle
		return fixedLeftDutyRaitoForConstantSpeed;
	}
}

float Navigation::calculateControlledRightDutyCycleByUsingPID() { //constant speed included
    float controlledRightDutyCycle = (initialDutyCycle + Kp*positionError + Ki*positionErrorIntegral + Kd*positionErrorDerivative) - leftOverFlowedPWMDutyCycle;
    float fixedRightDutyRaitoForConstantSpeed = controlledRightDutyCycle + isPIDSpeedControlOn*calculateControlledRightPWMByUsingPID(controlledRightDutyCycle);

    /*Debug.locate(60, 20);
    Debug.printf("Rcs:%.2f", fixedRightDutyRaitoForConstantSpeed);*/

	if (fixedRightDutyRaitoForConstantSpeed > 1.0f) {
		rightOverFlowedPWMDutyCycle = fixedRightDutyRaitoForConstantSpeed - 1.0f;
		return 1.0f;
	}
	else if (fixedRightDutyRaitoForConstantSpeed < -1.0f) {
		rightOverFlowedPWMDutyCycle = fixedRightDutyRaitoForConstantSpeed + 1.0f;
		return -1.0f;
	}
	else {
		rightOverFlowedPWMDutyCycle = 0; //reset overFlowedPWMDutyCycle
		return fixedRightDutyRaitoForConstantSpeed;
	}
}

void Navigation::writePWMDutyCycleFromPIDToBothWheel() {
    calculateErrorVariables();

    if (motorStopFlag == 1) {
		LeftMotorPtr->motorBreak();
		RightMotorPtr->motorBreak();
    }
    else{
		if (calculateControlledLeftDutyCycleByUsingPID() >= 0) { //if calculated PWM duty cycle is positive
			if (LeftMotorPtr->getMotorDirection() == 0)			 //check the wheel direction, if the wheel is going backward
				LeftMotorPtr->setMotorDirection(1);				 // set the direction to forward
			LeftMotorPtr->setPWMDutyCycle(calculateControlledLeftDutyCycleByUsingPID());
		}
		else {													//if calculated PWM duty cycle is negative 
			if (LeftMotorPtr->getMotorDirection() == 1)			//check the wheel direction, if the wheel is going forward 
				LeftMotorPtr->setMotorDirection(0);				//set the direction to backward
			LeftMotorPtr->setPWMDutyCycle(-calculateControlledLeftDutyCycleByUsingPID());
		}

		if (calculateControlledRightDutyCycleByUsingPID() >= 0) {//if calculated PWM duty cycle is positive
			if (RightMotorPtr->getMotorDirection() == 0)		 //check the wheel direction, if the wheel is going forward 
				RightMotorPtr->setMotorDirection(1);			 // set the direction to forward
			RightMotorPtr->setPWMDutyCycle(calculateControlledRightDutyCycleByUsingPID());
		}
		else {													//if calculated PWM duty cycle is negative 
			if (RightMotorPtr->getMotorDirection() == 1)		//check the wheel direction, if the wheel is going forward 
				RightMotorPtr->setMotorDirection(0);			//set the direction to backward
			RightMotorPtr->setPWMDutyCycle(-calculateControlledRightDutyCycleByUsingPID());
		}
    }
}

void Navigation::refreshDutyCycleForBothWheel() {
    ticker2ForRefreshPWMDutyCycle.attach(callback(this, &Navigation::writePWMDutyCycleFromPIDToBothWheel), PID_CALCULATION_TIME_DURATION);
}

void  Navigation::detachRefreshDutyCycleForBothWheel() {
    ticker2ForRefreshPWMDutyCycle.detach();
}

float Navigation::calculateControlledLeftPWMByUsingPID(float PWMFromTurningPID) {
	float temReturnValue = 0;
    leftSpeedError = LeftMotorPtr->getTheoreticalSpeed(PWMFromTurningPID) - LeftMotorPtr->getInstantaneousSpeed();
    /*Debug.locate(0, 0);
    Debug.printf("LPWM:%.2f", PWMFromTurningPID);
    Debug.locate(0, 10);
    Debug.printf("LTS:%.2f", getTheoreticalSpeed(PWMFromTurningPID));*/

    /*Debug.locate(0, 0);
    Debug.printf("LSE:%.2f", leftSpeedError);
    Debug.locate(0, 10);
    Debug.printf("LC:%.2f", leftPWMControlValue);*/

    if (leftSpeedError > 0.0f) {// speed is not enough
        leftPWMControlValue +=  (KP_OF_MOTOR_SPEED*leftSpeedError + KI_OF_MOTOR_SPEED*leftSpeedErrorIntegral + KD_OF_MOTOR_SPEED*leftSpeedErrorDerivative) / DUTY_CYCLE_OF_PWM_TO_SPEED_RATIO;
        lastLeftPWMControlValue = leftPWMControlValue;
		temReturnValue = leftPWMControlValue;
    }
    else if (leftSpeedError < 0.0f) {// speed is too much
        leftPWMControlValue += (KP_OF_MOTOR_SPEED*leftSpeedError + KI_OF_MOTOR_SPEED*leftSpeedErrorIntegral + KD_OF_MOTOR_SPEED*leftSpeedErrorDerivative) / DUTY_CYCLE_OF_PWM_TO_SPEED_RATIO;
        lastLeftPWMControlValue = leftPWMControlValue;
		temReturnValue = leftPWMControlValue;
    }
    else {
		temReturnValue = lastLeftPWMControlValue; //keep last speed
    }

    //calculate error derivative:
    leftSpeedErrorDerivative = (leftSpeedError-leftLastSpeedError)/ DELTA_TIME_FOR_SPEED_CALCULATION;

    //calculate error integral:
    leftSpeedErrorIntegral = leftLastSpeedErrorIntegral + leftSpeedError*PID_CALCULATION_TIME_DURATION; //PID_CALCULATION_TIME_DURATION is the sampling time
	if (leftSpeedErrorIntegral > SPEED_PID_INTEGRAL_UPPER_LIMIT)
		leftSpeedErrorIntegral = SPEED_PID_INTEGRAL_UPPER_LIMIT;
	if (leftSpeedErrorIntegral < SPEED_PID_INTEGRAL_LOWER_LIMIT)
		leftSpeedErrorIntegral = SPEED_PID_INTEGRAL_LOWER_LIMIT;
                                                                                                                                      /*refresh variables*/
    leftLastSpeedError = leftSpeedError;
    leftLastSpeedErrorIntegral = leftSpeedErrorIntegral;
	return temReturnValue;
}

float Navigation::calculateControlledRightPWMByUsingPID(float PWMFromTurningPID) {
	float temReturnValue = 0;
	rightSpeedError = RightMotorPtr->getTheoreticalSpeed(PWMFromTurningPID) - RightMotorPtr->getInstantaneousSpeed();

    /*Debug.locate(60, 0);
    Debug.printf("RPWM:%.2f", PWMFromTurningPID);

    Debug.locate(60, 10);
    Debug.printf("RTS:%.2f", getTheoreticalSpeed(PWMFromTurningPID));*/

    /*Debug.locate(60, 0);
    Debug.printf("RSE:%.2f", rightSpeedError);

    Debug.locate(60, 10);
    Debug.printf("RC:%.2f", rightPWMControlValue);*/

    if (rightSpeedError > 0.0f) {// speed is not enough
        rightPWMControlValue +=  (KP_OF_MOTOR_SPEED*rightSpeedError + KI_OF_MOTOR_SPEED*rightSpeedErrorIntegral + KD_OF_MOTOR_SPEED*rightSpeedErrorDerivative) / DUTY_CYCLE_OF_PWM_TO_SPEED_RATIO;
        lastRightPWMControlValue = rightPWMControlValue;
		temReturnValue = rightPWMControlValue;
    }
    else if (rightSpeedError < 0.0f) {// speed is too much
        rightPWMControlValue += (KP_OF_MOTOR_SPEED*rightSpeedError + KI_OF_MOTOR_SPEED*rightSpeedErrorIntegral + KD_OF_MOTOR_SPEED*rightSpeedErrorDerivative) / DUTY_CYCLE_OF_PWM_TO_SPEED_RATIO;
        lastRightPWMControlValue = rightPWMControlValue;
		temReturnValue = rightPWMControlValue;
    }
    else {
		temReturnValue = lastRightPWMControlValue;//keep last speed
    }
	//calculate error derivative:
	rightSpeedErrorDerivative = (rightSpeedError - rightLastSpeedError) / DELTA_TIME_FOR_SPEED_CALCULATION;

	//calculate error integral:
	rightSpeedErrorIntegral = rightLastSpeedErrorIntegral + rightSpeedError*PID_CALCULATION_TIME_DURATION; //PID_CALCULATION_TIME_DURATION is the sampling time
	if (rightSpeedErrorIntegral > SPEED_PID_INTEGRAL_UPPER_LIMIT)
		rightSpeedErrorIntegral = SPEED_PID_INTEGRAL_UPPER_LIMIT;
	if (rightSpeedErrorIntegral < SPEED_PID_INTEGRAL_LOWER_LIMIT)
		rightSpeedErrorIntegral = SPEED_PID_INTEGRAL_LOWER_LIMIT;

	/*refresh variables*/
	rightLastSpeedError = rightSpeedError;
	rightLastSpeedErrorIntegral = rightSpeedErrorIntegral;
	return temReturnValue;
}

void Navigation::balanceMotorDutyCycleToSpeedConversionRate() {
	float lowLRSpeedRatio, highLRSpeedRatio;
	float sumLowLRSpeedRatio = 0; 
	float sumHighLRSpeedRatio = 0;
	float temPWMDutyCycle = 0.15f;

	for (int i = 0; i < 6; ++i) {
		LeftMotorPtr->setPWMDutyCycle(temPWMDutyCycle);
		RightMotorPtr->setPWMDutyCycle(temPWMDutyCycle);
		wait(0.4);
		sumLowLRSpeedRatio +=  LeftMotorPtr->getAccurateInstantaneousSpeed()/ RightMotorPtr->getAccurateInstantaneousSpeed();
		temPWMDutyCycle += 0.04f;
	}
	lowLRSpeedRatio = sumLowLRSpeedRatio / 6.0f;

	temPWMDutyCycle += 0.1f;
	for (int i = 0; i < 14; ++i) {
		LeftMotorPtr->setPWMDutyCycle(temPWMDutyCycle);
		RightMotorPtr->setPWMDutyCycle(temPWMDutyCycle);
		wait(0.4);
		sumHighLRSpeedRatio += LeftMotorPtr->getAccurateInstantaneousSpeed()/RightMotorPtr->getAccurateInstantaneousSpeed();
		temPWMDutyCycle += 0.04f;
	}
	highLRSpeedRatio = sumHighLRSpeedRatio / 14.0f;

	LeftMotorPtr->setLRMotorsSpeedRatio(1.0f,1.0f);
	RightMotorPtr->setLRMotorsSpeedRatio(highLRSpeedRatio, lowLRSpeedRatio);

	Debug.locate(0, 0);
	Debug.printf("setLRMotorsSpeedRatio:");
	Debug.locate(0, 15);
	Debug.printf("H1:%.2f, L1:%.2f", highLRSpeedRatio, lowLRSpeedRatio);
	wait(1.0);
}

void Navigation::setKPID(float kp, float ki, float kd) {
	Kp = kp;
	Ki = ki;
	Kd = kd;
}

void Navigation::setInitialPWMDutyCycle(float IPD) {
	initialDutyCycle = IPD;
}

void Navigation::changePWMDutyCycleRunOnSlope() {
	if ((LeftMotorPtr->getInstantaneousSpeed() > 1.2f*LeftMotorPtr->getTheoreticalSpeed(LeftMotorPtr->getPWMDutyCycle())) && (RightMotorPtr->getInstantaneousSpeed() > 1.2f*RightMotorPtr->getTheoreticalSpeed(RightMotorPtr->getPWMDutyCycle()))) {//if current speed is larger than 1.3 times than what it should have, reduce PWM duty cycle
		setInitialPWMDutyCycle(0.65f*originalInitialDutyCycle);//was 0.7
	}	//if speed is too high when it's going down the slope, decelerate to keep original speed
	if ((RightMotorPtr->getInstantaneousSpeed()<0.8f*RightMotorPtr->getTheoreticalSpeed(initialDutyCycle))&& (initialDutyCycle<1)){// if initialDutyCycle >=1, can't accelerate anymore
		setInitialPWMDutyCycle(originalInitialDutyCycle);
	}	//if going down the slope is ended, get the PWM duty Cycle to original one to gain orignal speed on flat surface
	if ((RightMotorPtr->getInstantaneousSpeed()<0.7f*RightMotorPtr->getTheoreticalSpeed(initialDutyCycle))&& (LeftMotorPtr->getInstantaneousSpeed()<0.7f*LeftMotorPtr->getTheoreticalSpeed(initialDutyCycle))){
		setInitialPWMDutyCycle(1.25f*originalInitialDutyCycle);
	}	//if speed is too low when it's going up the slope, accelerate to keep original speed
}

//Race Functions:
void Navigation::showActualMileage() {
	calculateActualMileage();
	Debug.cls();
	Debug.locate(0, 0);
	Debug.printf("Actual Mileage:");
	Debug.locate(0, 20);
	Debug.printf("%f", actualMileage);
}

void Navigation::calculateActualMileage(){
	actualMileage += (LeftMotorPtr->mileageInMeters() + RightMotorPtr->mileageInMeters()) / 2;
}

// Debug functions:
float round(float r) {
	return (r > 0.0f) ? floor(r + 0.5f) : ceil(r - 0.5f);
}

float getNumTo0D1Decimal(float num) { // cut the number to 1 decimal point
	return round(num * 10) / 10.0f;
}

float getNumTo0D01Decimal(float num) { // cut the number to 1 decimal point
	return round(num * 100) / 100.0f;
}

float getNumTo0D001Decimal(float num) { // cut the number to 1 decimal point
	return round(num * 1000) / 1000.0f;
}

void Navigation::setSpeedByUsingPotentiometerAndDisplay() {
	Debug.cls();
	while (1) {
		LeftMotorPtr->setPWMDutyCycle(getNumTo0D1Decimal(PotentiometerPtr->getLeftPotRatio()));
		RightMotorPtr->setPWMDutyCycle(getNumTo0D1Decimal(PotentiometerPtr->getRightPotRatio()));
		Debug.locate(0, 0);
		Debug.printf("LPWM %.2f", getNumTo0D1Decimal(PotentiometerPtr->getLeftPotRatio()));
		Debug.locate(60, 0);
		Debug.printf("LSPD %.2f", LeftMotorPtr->getAccurateInstantaneousSpeed());

		Debug.locate(0, 15);
		Debug.printf("RPWM %.2f", getNumTo0D1Decimal(PotentiometerPtr->getRightPotRatio()));
		Debug.locate(60, 15);
		Debug.printf("RSPD %.2f", RightMotorPtr->getAccurateInstantaneousSpeed());
	}
}

void Navigation::showSpeedAndPWMDutyCycle() {
	Debug.locate(0, 0);
	Debug.printf("LPWM %.2f", LeftMotorPtr->getPWMDutyCycle());
	Debug.locate(0, 10);
	Debug.printf("LSPD %.2f", LeftMotorPtr->getAccurateInstantaneousSpeed());

	Debug.locate(60, 0);
	Debug.printf("RPWM %.2f", RightMotorPtr->getPWMDutyCycle());
	Debug.locate(60, 10);
	Debug.printf("RSPD %.2f", RightMotorPtr->getAccurateInstantaneousSpeed());
}

float Navigation::getPositionError() {
	return positionError;
}

void Navigation::setKPIDByUsingPotAndJoy() {

	wait(0.3);// need it, in first 0.? second the centre sensor will turned to 1 for some reason.
	int cursor = 1;
	bool settingDone = 0;
	Debug.cls();
	wait(0.1); //clear the screen for 0.5 s
	while (1) {
		// cursor list: when the cursor is on 1:Kp, 2:Ki 3:Kd 4:PWM
		if (settingDone == 1) { break; }
		while (1) {
			if (JoystickPtr->centerJoyPushed()) {
				settingDone = 1;
				Debug.cls();
				Debug.locate(0, 0);
				Debug.printf("Kp:%.2f, Ki:%.2f, Kd:%.3f", Kp, Ki, Kd);
				Debug.locate(0, 20);
				Debug.printf("PWM:%.2f", initialDutyCycle);
				wait(2);
				break;
			}
			if (JoystickPtr->upJoyPushed()) { // cancel setting
				Kp = KP;
				Ki = KI;
				Kd = KD;
				initialDutyCycle = INITIAL_DUTY_CYCLE;
				originalInitialDutyCycle = INITIAL_DUTY_CYCLE;
				settingDone = 1;
				Debug.cls();
				Debug.locate(0, 0);
				Debug.printf("Kp:%.2f, Ki:%.2f, Kd:%.3f", Kp, Ki, Kd);
				Debug.locate(0, 20);
				Debug.printf("PWM:%.2f", initialDutyCycle);
				wait(2);
				break;
			}
			if (JoystickPtr->leftJoyPushed()) {
				if (cursor == 1) { cursor = 4;  Debug.cls(); wait(0.2); }
				else if (cursor == 2) {
					cursor = 1; Debug.cls(); wait(0.2);
				}
				else if (cursor == 3) {
					cursor = 2; Debug.cls(); wait(0.2);
				}
				else {
					cursor = 3; Debug.cls(); wait(0.2);
				}
			}

			Debug.locate(5, 0);
			Debug.printf("Kp:%.2f", Kp);

			Debug.locate(60, 0);
			Debug.printf("Ki:%.2f", Ki);

			Debug.locate(5, 15);
			Debug.printf("Kd:%.3f", Kd);

			Debug.locate(60, 15);
			Debug.printf("PWM:%.2f", initialDutyCycle);


			if (cursor == 1) {//KP
				Debug.locate(0, 0);
				Debug.printf("*");
			}
			if (cursor == 2) {//KI
				Debug.locate(55, 0);
				Debug.printf("*");
			}
			if (cursor == 3) {//KD
				Debug.locate(0, 15);
				Debug.printf("*");
			}
			if (cursor == 4) { //PWM
				Debug.locate(55, 15);
				Debug.printf("*");
			}
			// setting KP/I/D AND initialDutyCycle
			if (cursor == 1) {
				Kp = getNumTo0D01Decimal(1.5f*PotentiometerPtr->getLeftPotRatio());
			}
			else if (cursor == 2) {
				Ki = getNumTo0D01Decimal(0.5f*PotentiometerPtr->getLeftPotRatio());
			}
			else if (cursor == 3) {
				Kd = getNumTo0D001Decimal(0.2f*PotentiometerPtr->getLeftPotRatio());
			}
			else if (cursor == 4) {
				initialDutyCycle = getNumTo0D01Decimal(PotentiometerPtr->getLeftPotRatio());
				originalInitialDutyCycle = initialDutyCycle;
			}

		}
		
	}
}

void Sensor::showSensorState() {
	while (true){
		Debug.locate(0, 10);
		Debug.printf("L2:%d L1:%d CL:%d CR:%d R1:%d R2:%d", positionLeft2(),  positionLeft1(),  positionCenterL(),  positionCenterR(),  positionRight1(),  positionRight2());
		Debug.locate(0, 20);
		Debug.printf("\t%.2f %.2f %.2f %.2f %.2f %.2f", getSensorInputLeft2Percentage(),  getSensorInputLeft1Percentage(),  getsensorInputCenterLPercentage(),  getsensorInputCenterRPercentage(),  getSensorInputRight1Percentage(),  getSensorInputRight2Percentage());
	}
}