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
	private static float farLeftValue;
	private static float middleLeftValue;
	private static float middleRightValue;
	private static float farRightValue;
	
	// Declare black and white storage variables.
	private static float black;
	private static float white;
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
			
			farLeftValue = farLeftSample[0];
			middleLeftValue = middleLeftSample[0];
			middleRightValue = middleRightSample[0];
			farRightValue = farRightSample[0];
			
			errorCalc(farLeftValue, middleLeftValue, middleRightValue, farRightValue);
			
			LCD.drawString("FL: " + farLeftSample[0], 0, 0);
			LCD.drawString("ML: " + middleLeftSample[0], 0, 1);
			LCD.drawString("MR: " + middleRightSample[0], 0, 2);
			LCD.drawString("FR: " + farRightSample[0], 0, 3);
			
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
			
			Delay.msDelay(1000);
		}
	}
	
	public static float errorCalc(float valFL, float valML, float valMR, float valFR) {
		float error = 0;
		float FLError;
		float MLError;
		float MRError;
		float FRError;
		
		float valFLDiff = Math.abs(valFL - white);
		if (valFLDiff > 0.02) { FLError = (valFLDiff - scaleDifference) * 100;}
		else {FLError = 0;}
		
		float valMLDiff = Math.abs(valML - black);
		if (valFLDiff > 0.02) { MLError = (valFLDiff - scaleDifference) * 100;}
		else {MLError = 0;}
		
		float valMRDiff = Math.abs(valMR - black);
		if (valFLDiff > 0.02) { MRError = (valFLDiff - scaleDifference) * 100;}
		else {MRError = 0;}
		
		float valFRDiff = Math.abs(valFR - white);
		if (valFLDiff > 0.02) { FRError = (valFLDiff - scaleDifference) * 100;}
		else {FRError = 0;}
		
		error = MRError + FRError - valFLDiff - valMLDiff;
		
		return error;
	}

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

		farLeftColor.fetchSample(farLeftSample, 0);
		middleLeftColor.fetchSample(middleLeftSample, 0);
		middleRightColor.fetchSample(middleRightSample, 0);
		farRightColor.fetchSample(farRightSample, 0);
		
		black = (farLeftSample[0] + middleLeftSample[0] + middleRightSample[0] + farRightSample[0]) / 4;

		LCD.drawString("Black calibrated.", 0, 0);
		LCD.drawString("Press UP to set", 0, 1);
		LCD.drawString("white color.", 0, 2);
		Button.UP.waitForPressAndRelease();
		
		farLeftColor.fetchSample(farLeftSample, 0);
		middleLeftColor.fetchSample(middleLeftSample, 0);
		middleRightColor.fetchSample(middleRightSample, 0);
		farRightColor.fetchSample(farRightSample, 0);
		
		white = (farLeftSample[0] + middleLeftSample[0] + middleRightSample[0] + farRightSample[0]) / 4;
		
		scaleDifference = white - black;
		
		// leftWhiteThreshold = leftWhite * 0.2f;
		// rightWhiteThreshold = rightWhite * 0.2f;

		LCD.clear();
		LCD.drawString("Black: " + black, 0, 1);
		LCD.drawString("White: " + white, 0, 2);
		LCD.drawString("Diff: " + scaleDifference, 0, 2);

		Delay.msDelay(5000);
	}
}