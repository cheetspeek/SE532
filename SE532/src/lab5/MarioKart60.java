package lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class MarioKart60 implements TimerListener {
	private static SensorModes leftSensor;
	private static SensorModes rightSensor;
	private static SampleProvider leftColor;
	private static SampleProvider rightColor;

	private static Port portA;
	private static Port portD;
	private static Port port1;
	private static Port port4;
	public static RegulatedMotor rightWheel;
	private static RegulatedMotor leftWheel;
	private static float[] leftSample;
	private static float[] rightSample;
	private static Timer timer;

	private static float leftBlack;
	private static float leftWhite;
	private static float rightBlack;
	private static float rightWhite;
	
	private static int threshold;

	private static int speed;
	private static int acceleration;

	public static void main(String[] args) {
		setup();

		calibrate();

		timer.start();

		//rightWheel.backward();
		//leftWheel.backward();
		//rightWheel.forward();
		//leftWheel.forward();

		/*
		while (! Button.DOWN.isDown()) {

			if ( (Math.abs(rightSample[0] - rightWhite) < 0.2) ) {
				leftWheel.setSpeed(speed + 30);
				leftWheel.backward();
				//leftWheel.setSpeed(speed + 30);
				//rightWheel.setSpeed(5);
			}
			else if ( (Math.abs(leftSample[0] - leftWhite) < 0.2) ) {
				rightWheel.setSpeed(speed + 30);
				rightWheel.backward();
				//rightWheel.setSpeed(speed + 30);
				//leftWheel.setSpeed(5);
			}
			else {
				rightWheel.setSpeed(speed);
				leftWheel.setSpeed(speed);
			}

		}
		 */

		Button.DOWN.waitForPressAndRelease();

		rightWheel.close();
		leftWheel.close();
	}

	public static void setup() {
		portA = LocalEV3.get().getPort(MotorPort.A.getName());
		rightWheel = new EV3LargeRegulatedMotor(portA);

		portD = LocalEV3.get().getPort(MotorPort.D.getName());
		leftWheel = new EV3LargeRegulatedMotor(portD);

		RegulatedMotor[] list = {leftWheel};

		port1 = LocalEV3.get().getPort("S1");
		leftSensor = new EV3ColorSensor(port1);

		port4 = LocalEV3.get().getPort("S4");
		rightSensor = new EV3ColorSensor(port4);

		leftColor = leftSensor.getMode("Red");
		rightColor = rightSensor.getMode("Red");

		leftSample = new float[leftSensor.sampleSize()];
		rightSample = new float[rightSensor.sampleSize()];
		
		threshold = 44;

		timer = new Timer(1, new MarioKart60());

		//speed = 375;
		//acceleration = 1500;
		speed = 300;
		acceleration = 1000;
		rightWheel.setSpeed(speed);
		leftWheel.setSpeed(speed);
		rightWheel.setAcceleration(acceleration);
		leftWheel.setAcceleration(acceleration);

		rightWheel.synchronizeWith(list);
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

		LCD.clear();
		LCD.drawString("White calibrated.", 0, 0);
		LCD.drawString("LB: " + leftBlack + ", LW: " + leftWhite, 0, 1);
		LCD.drawString("RB: " + rightBlack + ", RW: " + rightWhite, 0, 2);

		Delay.msDelay(5000);

	}

	@Override
	public void timedOut() {
		rightWheel.backward();
		leftWheel.backward();
		
		leftColor.fetchSample(leftSample, 0);
		rightColor.fetchSample(rightSample, 0);

		LCD.drawString("Sample: " + leftSample[0], 0, 3);
		
		/*
		//Front wheel drive
//		if ( (Math.abs(leftSample[0] - leftWhite) < 0.2) && (Math.abs(rightSample[0] - rightWhite) < 0.2) ) {
//			LCD.drawString("Stay straight", 0, 5);
//			rightWheel.setSpeed(speed);
//			leftWheel.setSpeed(speed);
//		}
		if ( (Math.abs(leftSample[0] - leftBlack) < 0.2) ){
			LCD.drawString("Turning Right", 0, 5);
			//leftWheel.setSpeed(speed + 30);
			rightWheel.setSpeed(10);
		}
		else if ( (Math.abs(rightSample[0] - rightBlack) < 0.2) ) {
			LCD.drawString("Turning Left", 0, 5);
			//rightWheel.setSpeed(speed + 30);
			leftWheel.setSpeed(10);
		} 
		else {
			rightWheel.setSpeed(speed);
			leftWheel.setSpeed(speed);
		}
		*/
		
		
		//One sensor, front wheel drive
		int multiplier = -3;
		int rotateVal = (int) ((threshold - leftSample[0]) * multiplier);
		LCD.drawString("RotateVal: " + rotateVal, 0, 4);
		
		leftWheel.startSynchronization();
		leftWheel.rotate(rotateVal, true);
		rightWheel.rotate(rotateVal, true);
		leftWheel.endSynchronization();
		
		
		/*
		//Rear wheel drive
		//int angle = -82;
		if ( (Math.abs(rightSample[0] - rightWhite) < 0.2) ) {
			rightWheel.setSpeed(speed + 25);
			leftWheel.setSpeed(10);

			//rotate(leftWheel, rightWheel, angle);
		}
		else if ( (Math.abs(leftSample[0] - leftWhite) < 0.2) ) {
			leftWheel.setSpeed(speed + 25);
			rightWheel.setSpeed(10);

			//rotate(rightWheel, leftWheel, angle);
		}
		 */
	}
}