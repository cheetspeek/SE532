package lab4;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class EnExTee implements TimerListener {
	private static EV3UltrasonicSensor sensor;
	private static Port port;
	private static Port port2;
	private static Port port3;
	private static Port port4;
	public static RegulatedMotor m1;
	private static RegulatedMotor m2;
	private static RegulatedMotor spinSensorMotor;
	private static SampleProvider USsensor;
	private static float[] sample;
	private static Timer timer;

	protected static int angle = 0;
	
	protected static float distance = 0;
	public static float minAngle = -1;
	public static int tachoVal = 0;
	
	protected static int recalVal = 7;

	protected static Boolean isObstructed = false;

	public static void main(String[] args) {
		setup();

		int iterations = 0;
		
		calibrate();

		// loop guard and if must be the same
		while (iterations <= 32) {
			if (isObstructed == false) {
				// Sends GoBot to final block
				if (iterations == 32) {
					//angle = 1600;
					angle = 1690;
					rotate(m1,m2,angle);
					iterations++;
				}
				// Sends GoBot to next block
				else if (iterations % 2 == 0) {
					//angle = 780;
					angle = 825;
					rotate(m1,m2,angle);
					iterations++;
				}
				// Turns GoBot and checks for re-calibrate 
				else {
					if (iterations % recalVal == 0) {
						recalVal = recalVal + 8;
						recalibrate();
					}
					//angle = -165;
					angle = -166;
					rotate(m1,m2,angle);
					iterations++;
				}
			}
		}

		m1.close();
		m2.close();
	}

	public static void setup() {
		port = LocalEV3.get().getPort(MotorPort.A.getName());
		m1 = new EV3LargeRegulatedMotor(port);

		port2 = LocalEV3.get().getPort(MotorPort.B.getName());
		m2 = new EV3LargeRegulatedMotor(port2);

		RegulatedMotor[] list = {m2};

		port3 = LocalEV3.get().getPort("S1");
		sensor = new EV3UltrasonicSensor(port3);
		
		port4 = LocalEV3.get().getPort(MotorPort.C.getName());
		spinSensorMotor = new EV3LargeRegulatedMotor(port4);
		spinSensorMotor.setSpeed(300);
		spinSensorMotor.setAcceleration(100);
		
		USsensor = sensor.getMode("Distance");
		sample = new float[USsensor.sampleSize()];

		timer = new Timer(1, new EnExTee());

		LCD.drawString("Running.", 0, 0);

		// timer.start();

		m1.setSpeed(750);
		m2.setSpeed(750);
		m1.setAcceleration(300);
		m2.setAcceleration(300);

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
		USsensor.fetchSample(sample, 0);
		distance = sample[0];
		Sound.beep();
		LCD.drawString("Distance: " + distance, 0, 1);
		Delay.msDelay(3000);
		angle = -165;
		rotate(m1,m2,angle);
	}
	
	public static void recalibrate() {
		LCD.drawString("Recalibrating...", 0, 0);
		
		timer.start();
		
		USsensor.fetchSample(sample, 0);
		spinSensorMotor.rotate(82);
		spinSensorMotor.rotate(-165);
		spinSensorMotor.rotate(82);
		
		timer.stop();
		
		//recal angle
		LCD.drawString("Fixing the angle...", 0, 0);
		LCD.drawString("minAngle: " + minAngle, 0, 1);
		
		float angleDiff = sample[0] - minAngle;
		double angleRotations = angleDiff / 0.258;
		int angleDegrees = ((int) (angleRotations * 360)) / -1;
		LCD.drawString("angleDegrees: " + angleDegrees, 0, 2);
		
		if (angleDegrees >= 0) {
			spinLeftWheel(angleDegrees);
		} else {
			spinRightWheel(angleDegrees);
		}
		
		USsensor.fetchSample(sample, 0);
		float difference = distance - sample[0];
		Sound.beep();
		LCD.drawString("Distance: " + difference, 0, 4);
		Delay.msDelay(3000);
		double rotations = difference / 0.258;
		int degrees = ((int) (rotations * 360)) / -1;
		LCD.drawString("Degrees: " + degrees, 0, 5);
		rotateBack(m1,m2,degrees);
		USsensor.fetchSample(sample, 0);
		LCD.drawString("New distance: " + sample[0], 0, 6);
	}
	
	private static void spinLeftWheel(int angle) {
		m2.setAcceleration(150);
		m2.rotate(angle, true);
		m2.waitComplete();
		m2.setAcceleration(300);
	}
	
	private static void spinRightWheel(int angle) {
		m1.setAcceleration(150);
		m1.rotate(angle, true);
		m1.waitComplete();
		m1.setAcceleration(300);
	}
	
	public static void rotateBack(RegulatedMotor m1, RegulatedMotor m2, int angle) {
		m1.setAcceleration(150);
		m2.setAcceleration(150);
		m1.startSynchronization();
		m1.rotate(angle, true);
		m2.rotate(angle, true);
		m1.endSynchronization();
		m1.waitComplete();
		m2.waitComplete();
		m1.setAcceleration(300);
		m2.setAcceleration(300);
	}
	
	@Override
	public void timedOut() {
		USsensor.fetchSample(sample, 0);

		if (minAngle == -1.0) {
			minAngle = sample[0];
		} else if (sample[0] < minAngle) {
			minAngle = sample[0];
			//tachoVal = m1.getTachoCount();
		}
	}
}