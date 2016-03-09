package lab5;

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

public class MarioKart60 {
	private static SensorModes leftSensor;
	private static SensorModes rightSensor;
	private static SampleProvider leftColor;
	private static SampleProvider rightColor;

	private static Port portA;
	private static Port portD;
	private static Port port1;
	private static Port port4;
	private static UnregulatedMotor rightWheel;
	private static UnregulatedMotor leftWheel;
	private static float[] leftSample;
	private static float[] rightSample;

	private static float leftBlack;
	private static float leftWhite;
	private static float rightBlack;
	private static float rightWhite;
	
	private static float leftWhiteThreshold;
	private static float rightWhiteThreshold;
	
	private static final int SPEED = 80;
	private static final int TURN_SPEED = 25;
	
	public static void main(String[] args) {
		setup();

		calibrate();
		
		rightWheel.setPower(SPEED);
		leftWheel.setPower(SPEED);
		
		rightWheel.backward();
		leftWheel.backward();
		
		while (! Button.DOWN.isDown()) {
			leftColor.fetchSample(leftSample, 0);
			rightColor.fetchSample(rightSample, 0);		
			
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
			leftWheel.backward();
		}
	}

	public static void setup() {
		portA = LocalEV3.get().getPort(MotorPort.A.getName());
		rightWheel = new UnregulatedMotor(portA);

		portD = LocalEV3.get().getPort(MotorPort.D.getName());
		leftWheel = new UnregulatedMotor(portD);

		port1 = LocalEV3.get().getPort("S1");
		leftSensor = new EV3ColorSensor(port1);

		port4 = LocalEV3.get().getPort("S4");
		rightSensor = new EV3ColorSensor(port4);
    	
		leftColor = leftSensor.getMode("Red");
		rightColor = rightSensor.getMode("Red");

		leftSample = new float[leftSensor.sampleSize()];
		rightSample = new float[rightSensor.sampleSize()];
	}

	public static void calibrate() {
		LCD.drawString("Press UP to set", 0, 0);
		LCD.drawString("black color.", 0, 1);
		Button.UP.waitForPressAndRelease();

		leftColor.fetchSample(leftSample, 0);
		leftBlack = leftSample[0];

		rightColor.fetchSample(rightSample, 0);
		rightBlack = rightSample[0];

		LCD.drawString("Black calibrated.", 0, 0);
		LCD.drawString("Press UP to set", 0, 1);
		LCD.drawString("white color.", 0, 2);
		Button.UP.waitForPressAndRelease();

		leftColor.fetchSample(leftSample, 0);
		leftWhite = leftSample[0];

		rightColor.fetchSample(rightSample, 0);
		rightWhite = rightSample[0];
		
		leftWhiteThreshold = leftWhite * 0.2f;
		rightWhiteThreshold = rightWhite * 0.2f;

		LCD.clear();
		LCD.drawString("White calibrated.", 0, 0);
		LCD.drawString("LB: " + leftBlack + ", LW: " + leftWhite, 0, 1);
		LCD.drawString("RB: " + rightBlack + ", RW: " + rightWhite, 0, 2);

		Delay.msDelay(5000);
	}
}