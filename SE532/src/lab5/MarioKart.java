package lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
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

public class MarioKart implements TimerListener {
	private static SensorModes leftSensor;
	private static SensorModes rightSensor;
	private static SampleProvider leftColor;
	private static SampleProvider rightColor;
	
	private static Port port;
	private static Port port2;
	private static Port port3;
	private static Port port4;
	public static RegulatedMotor m1;
	private static RegulatedMotor m2;
	private static float[] leftSample;
	private static float[] rightSample;
	private static Timer timer;
	
	private static float black;
	private static float white;
	
	private static int speed;

	public static void main(String[] args) {
		setup();
		
		calibrate();
		
		timer.start();

		m1.forward();
		m2.forward();
		
		Button.DOWN.waitForPressAndRelease();

		m1.close();
		m2.close();
	}

	public static void setup() {
		port = LocalEV3.get().getPort(MotorPort.A.getName());
		m1 = new EV3LargeRegulatedMotor(port);

		port2 = LocalEV3.get().getPort(MotorPort.D.getName());
		m2 = new EV3LargeRegulatedMotor(port2);

		RegulatedMotor[] list = {m2};

		port3 = LocalEV3.get().getPort("S1");
		rightSensor = new EV3ColorSensor(port3);
		
		port4 = LocalEV3.get().getPort("S4");
		leftSensor = new EV3ColorSensor(port4);
		
		leftColor = leftSensor.getMode("Red");
		rightColor = rightSensor.getMode("Red");
		
		leftSample = new float[leftSensor.sampleSize()];
		rightSample = new float[rightSensor.sampleSize()];

		timer = new Timer(1, new MarioKart());
		speed = 350;

		m1.setSpeed(speed);
		m2.setSpeed(speed);
		m1.setAcceleration(900);
		m2.setAcceleration(900);

		m1.synchronizeWith(list);
	}

	public static void rotate(RegulatedMotor m1, RegulatedMotor m2, int angle) {
		m1.startSynchronization();
		m1.rotate(angle, true);
		m2.rotate(Math.abs(angle), true);
		m1.endSynchronization();
		m1.waitComplete();
		m2.waitComplete();
	}
	
	public static void calibrate() {
		LCD.drawString("Press UP to set", 0, 0);
		LCD.drawString("black color.", 0, 1);
		Button.UP.waitForPressAndRelease();
		
		leftColor.fetchSample(leftSample, 0);
		black = leftSample[0];
		
		LCD.drawString("Black calibrated.", 0, 0);
		LCD.drawString("Press UP to set", 0, 1);
		LCD.drawString("white color.", 0, 2);
		Button.UP.waitForPressAndRelease();
		
		leftColor.fetchSample(leftSample, 0);
		white = leftSample[0];
		
		LCD.clear();
		LCD.drawString("White calibrated.", 0, 0);
		LCD.drawString("Black value: " + black, 0, 2);
		LCD.drawString("White value: " + white, 0, 3);
		
		Delay.msDelay(5000);
		
	}
	/*
	private static void spinLeftWheel() {
		m2.setSpeed(400);
		Delay.msDelay(100);
		m2.setSpeed(150);
	}
	
	private static void spinRightWheel() {
		m1.setSpeed(400);
		Delay.msDelay(100);
		m1.setSpeed(150);
	}*/
	
	@Override
	public void timedOut() {
		leftColor.fetchSample(leftSample, 0);
		rightColor.fetchSample(rightSample, 0);
		
		if ( (Math.abs(rightSample[0] - black) < 0.2) ) {
			m2.setSpeed(speed + 50);
			m1.setSpeed(50);
			
		}
		
		else if ( (Math.abs(leftSample[0] - black) < 0.2) ) {
			m1.setSpeed(speed + 50);
			m2.setSpeed(50);
		}
		
		else {
			m1.setSpeed(speed);
			m2.setSpeed(speed);
		}	
	}
}