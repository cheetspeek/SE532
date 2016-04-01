package lab6;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class MarioKartPD {
	// Declare four color sensors.
	private static SensorModes farLeftSensor;
	private static SensorModes middleLeftSensor;
	private static SensorModes middleRightSensor;
	private static SensorModes farRightSensor;

	// Declare four color providers.
	private static SampleProvider farLeftColor;
	private static SampleProvider middleLeftColor;
	private static SampleProvider middleRightColor;
	private static SampleProvider farRightColor;

	// Declare servo ports.
	private static Port portA;
	private static Port portD;

	// Declare sensor ports.
	private static Port port1;
	private static Port port2;
	private static Port port3;
	private static Port port4;

	// Declare servo motors.
	private static UnregulatedMotor rightWheel;
	private static UnregulatedMotor leftWheel;

	// Declare sensor data arrays.
	private static float[] farLeftSample;
	private static float[] middleLeftSample;
	private static float[] middleRightSample;
	private static float[] farRightSample;

	// Declare sensor data variables.
	private static int farLeftValue;
	private static int middleLeftValue;
	private static int middleRightValue;
	private static int farRightValue;

	// Declare black and white storage variables.
	private static float farLeftBlack = 999;
	private static float middleLeftBlack = 999;
	private static float middleRightBlack = 999;
	private static float farRightBlack = 999;
	private static float farLeftWhite = -999;
	private static float middleLeftWhite = -999;
	private static float middleRightWhite = -999;
	private static float farRightWhite = -999;
	private static float farLeftStart;
	private static float middleLeftStart;
	private static float middleRightStart;
	private static float farRightStart;
	private static float farLeftDiff;
	private static float middleLeftDiff;
	private static float middleRightDiff;
	private static float farRightDiff;


	private static float scaleDifference;

	private static final int SPEED = 84; //80
	private static final int TURN_SPEED = 28; //25

	public static void main(String[] args) {
		setup();

		calibrate();

		LCD.clear();
		/*
		rightWheel.setPower(SPEED);
		leftWheel.setPower(SPEED);

		rightWheel.backward();
		leftWheel.backward();
		 */
		while (! Button.DOWN.isDown()) {
			farLeftColor.fetchSample(farLeftSample, 0);
			middleLeftColor.fetchSample(middleLeftSample, 0);
			middleRightColor.fetchSample(middleRightSample, 0);	
			farRightColor.fetchSample(farRightSample, 0);

			farLeftValue = (int) zeroToHundredCal(farLeftSample[0], farLeftWhite, farLeftBlack);
			middleLeftValue = (int) zeroToHundredCal(middleLeftSample[0], middleLeftWhite, middleLeftBlack);
			middleRightValue = (int) zeroToHundredCal(middleRightSample[0], middleRightWhite, middleRightBlack);
			farRightValue = (int) zeroToHundredCal(farRightSample[0], farRightWhite, farRightBlack);
			
			farLeftValue = correctValues(farLeftValue);
			middleLeftValue = correctValues(middleLeftValue);
			middleRightValue = correctValues(middleRightValue);
			farRightValue = correctValues(farRightValue);

			LCD.drawString("FL: " + farLeftValue, 0, 0);
			LCD.drawString("ML: " + middleLeftValue, 0, 1);
			LCD.drawString("MR: " + middleRightValue, 0, 2);
			LCD.drawString("FR: " + farRightValue, 0, 3);

			//int error = errorCalc(farLeftValue, middleLeftValue, middleRightValue, farRightValue);
			//LCD.drawString("Error: " + error, 0, 4);

			/*
			if ( rightSample[0] < rightWhiteThreshold ) {
				leftWheel.setPower(TURN_SPEED);
				rightWheel.setPower(SPEED);
			}
			else if ( leftSample[0] < leftWhiteThreshold  ) {
				rightWheel.setPower(TURN_SPEED);
				leftWheel.setPower(SPEED);
			}
			else {
				rightWheel.setPower(SPEED);
				leftWheel.setPower(SPEED);
			}

			rightWheel.backward();
			leftWheel.backward();*/

			
		}
	}
	
	public static int correctValues(int value) {
		if (value > 100) {
			value = 100;
		}
		else if (value < 0) {
			value = 0;
		}
		return value;
	}

	public static int middleErrorCalc(int left, int right) {
		return (int) ((left-right) * 0.33);
	}
	
	public static int outerErrorCalc(int outer, int inner) {
		return (int) (((outer-inner) * -0.5) + 68);
	}
	
	/*
	public static int errorCalc(int s1, int s2, int s3, int s4) {
		int totalError = 0;
		int middleError = (s2 - s3) / 2;
		int leftError = (100 - s1) / 2;
		int rightError = (100 - s4) / 2;
		
		
		if (s2 > s3) {
			//leftError = Math.abs(s3 - s4) / 2;
			totalError = (middleError + leftError) * -1;
		}
		else { // if(s2 < s3) - moving to right {
			// rightError = Math.abs(s1 - s2) / 2;
			totalError = (middleError + rightError);
		}
		
//		if (s2 > 86 && s3 > 86) {
//			totalError += 50;
//		}

		return totalError;
	}
	*/

	public static void setup() {
		portA = LocalEV3.get().getPort(MotorPort.A.getName());
		rightWheel = new UnregulatedMotor(portA);

		portD = LocalEV3.get().getPort(MotorPort.D.getName());
		leftWheel = new UnregulatedMotor(portD);

		port1 = LocalEV3.get().getPort("S1");
		farLeftSensor = new EV3ColorSensor(port1);

		port2 = LocalEV3.get().getPort("S2");
		middleLeftSensor = new EV3ColorSensor(port2);

		port3 = LocalEV3.get().getPort("S3");
		middleRightSensor = new EV3ColorSensor(port3);

		port4 = LocalEV3.get().getPort("S4");
		farRightSensor = new EV3ColorSensor(port4);

		farLeftColor = farLeftSensor.getMode("Red");
		middleLeftColor = middleLeftSensor.getMode("Red");
		middleRightColor = middleRightSensor.getMode("Red");
		farRightColor = farRightSensor.getMode("Red");

		farLeftSample = new float[farLeftSensor.sampleSize()];
		middleLeftSample = new float[middleLeftSensor.sampleSize()];
		middleRightSample = new float[middleRightSensor.sampleSize()];
		farRightSample = new float[farRightSensor.sampleSize()];
	}

	public static void calibrate() {
		LCD.drawString("Press UP to set", 0, 0);
		LCD.drawString("black color.", 0, 1);
		Button.UP.waitForPressAndRelease();

		findMinBlackValues();

		LCD.drawString("Black calibrated.", 0, 0);
		LCD.drawString("Press UP to set", 0, 1);
		LCD.drawString("white color.", 0, 2);
		Button.UP.waitForPressAndRelease();

		findMaxWhiteValues();

		//findStartValues();

		//findDifferences();

		LCD.clear();
		//LCD.drawString("Diff: " + scaleDifference, 0, 2);

		Delay.msDelay(5000);
	}


	public static void findMinBlackValues() {	
		for (int i = 0; i < 50; i++) {
			farLeftColor.fetchSample(farLeftSample, 0);
			if (farLeftSample[0] < farLeftBlack) {
				farLeftBlack = farLeftSample[0];
			}
		}

		for (int i = 0; i < 50; i++) {
			middleLeftColor.fetchSample(middleLeftSample, 0);
			if (middleLeftSample[0] < middleLeftBlack) {
				middleLeftBlack = middleLeftSample[0];
			}
		}

		for (int i = 0; i < 50; i++) {
			middleRightColor.fetchSample(middleRightSample, 0);
			if (middleRightSample[0] < middleRightBlack) {
				middleRightBlack = middleRightSample[0];
			}
		}

		for (int i = 0; i < 50; i++) {
			farRightColor.fetchSample(farRightSample, 0);
			if (farRightSample[0] < farRightBlack) {
				farRightBlack = farRightSample[0];
			}
		}
	}

	public static void findMaxWhiteValues() {
		for (int i = 0; i < 50; i++) {
			farLeftColor.fetchSample(farLeftSample, 0);
			if (farLeftSample[0] > farLeftWhite) {
				farLeftWhite = farLeftSample[0];
			}
		}

		for (int i = 0; i < 50; i++) {
			middleLeftColor.fetchSample(middleLeftSample, 0);
			if (middleLeftSample[0] > middleLeftWhite) {
				middleLeftWhite = middleLeftSample[0];
			}
		}

		for (int i = 0; i < 50; i++) {
			middleRightColor.fetchSample(middleRightSample, 0);
			if (middleRightSample[0] > middleRightWhite) {
				middleRightWhite = middleRightSample[0];
			}
		}

		for (int i = 0; i < 50; i++) {
			farRightColor.fetchSample(farRightSample, 0);
			if (farRightSample[0] > farRightWhite) {
				farRightWhite = farRightSample[0];
			}
		}
	}

	public static float zeroToHundredCal(float val, float max, float min) {		
		return ((val - min)/(max - min)) * 100;
	}

	/*
	public static void findStartValues() {
		farLeftColor.fetchSample(farLeftSample, 0);
		middleLeftColor.fetchSample(middleLeftSample, 0);
		middleRightColor.fetchSample(middleRightSample, 0);
		farRightColor.fetchSample(farRightSample, 0);

		farLeftStart = farLeftSample[0];
		middleLeftStart = middleLeftSample[0];
		middleRightStart = middleRightSample[0];
		farRightStart = farRightSample[0];
	}

	public static void findDifferences() {
		farLeftDiff = farLeftWhite - farLeftBlack;
		middleLeftDiff = middleLeftWhite - middleLeftBlack;
		middleRightDiff = middleRightWhite - middleRightBlack;
		farRightDiff = farRightWhite - farRightBlack;
	}
	 */
}