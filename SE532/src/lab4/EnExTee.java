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
// import lejos.utility.Timer;
// import lejos.utility.TimerListener;

public class EnExTee {
	private static EV3UltrasonicSensor sensor;
	private static Port port;
	private static Port port2;
	private static Port port3;
	private static RegulatedMotor m1;
	private static RegulatedMotor m2;
	private static SampleProvider UV;
	private static float[] sample;

	protected static int speed = 0;
	
	protected static float distance = 0;
	
	protected static int recalVal = 7;

	protected static Boolean isObstructed = false;

	public static void main(String[] args) {
		setup();

		int iterations = 0;
		
		calibrate();

		// loop guard and if must be the same
		while (iterations <= 24) {
			if (isObstructed == false) {
				// Sends GoBot to final block
				if (iterations == 24) {
					speed = 1600;
					rotate(m1,m2,speed);
					iterations++;
				}
				// Sends GoBot to next block
				else if (iterations % 2 == 0) {
					speed = 780;
					rotate(m1,m2,speed);
					iterations++;
				}
				// Turns GoBot and checks for re-calibrate 
				else {
					if (iterations % recalVal == 0) {
						recalVal = recalVal + 8;
						recalibrate();
					}
					speed = -165;
					rotate(m1,m2,speed);
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
		
		UV = sensor.getMode("Distance");
		sample = new float[UV.sampleSize()];

		// UVTimer listener = new UVTimer(port2, sensor);
		// Timer timer = new Timer(1, listener);

		LCD.drawString("Running.", 0, 0);

		// timer.start();

		m1.setSpeed(750);
		m2.setSpeed(750);
		m1.setAcceleration(300);
		m2.setAcceleration(300);

		m1.synchronizeWith(list);
	}

	public static void rotate(RegulatedMotor m1, RegulatedMotor m2, int speed) {
		m1.startSynchronization();
		m1.rotate(speed, true);
		m2.rotate(Math.abs(speed), true);
		m1.endSynchronization();
		m1.waitComplete();
		m2.waitComplete();
	}
	
	public static void calibrate() {
		UV.fetchSample(sample, 0);
		distance = sample[0];
		Sound.beep();
		LCD.drawString("Distance: " + distance, 0, 1);
		Delay.msDelay(3000);
		speed = -165;
		rotate(m1,m2,speed);
	}
	
	public static void recalibrate() {
		UV.fetchSample(sample, 0);
		double difference = (double) distance - sample[0];
		Sound.beep();
		LCD.drawString("Distance: " + difference, 0, 1);
		Delay.msDelay(3000);
		double rotations = difference / 0.258;
		int degrees = ((int) (rotations * 360)) / -1;
		LCD.drawString("Degrees: " + degrees, 0, 2);
		rotateBack(m1,m2,degrees);
		UV.fetchSample(sample, 0);
		LCD.drawString("New distance: " + sample[0], 0, 3);
	}
	
	public static void rotateBack(RegulatedMotor m1, RegulatedMotor m2, int speed) {
		m1.setAcceleration(150);
		m2.setAcceleration(150);
		m1.startSynchronization();
		m1.rotate(speed, true);
		m2.rotate(speed, true);
		m1.endSynchronization();
		m1.waitComplete();
		m2.waitComplete();
		m1.setAcceleration(300);
		m2.setAcceleration(300);
	}

	public static void interrupt() {
		m1.startSynchronization();
		m1.stop();
		m2.stop();
		m1.endSynchronization();
		m1.waitComplete();
		m2.waitComplete();
		// m1Rotate = m1.getTachoCount();
		// m2Rotate = m2.getTachoCount();
	}

	public static void restart() {
		/*
		m1.startSynchronization();
		m1.rotate(speed - m1Rotate, true);
		m2.rotate(Math.abs(speed) - m2Rotate, true);
		m1.endSynchronization();
		m1.waitComplete();
		m2.waitComplete();
		 */
	}
}
/*
class UVTimer implements TimerListener {

	Port port;
	EV3UltrasonicSensor s;
	float[] sample;
	SampleProvider UV;

	int ticks = 0;

	public UVTimer(Port port, EV3UltrasonicSensor s) {
		this.port = port;
		this.s = s;
		UV = s.getMode("Distance");
		s.enable();
		sample = new float[UV.sampleSize()];
	}

	@Override
	public void timedOut() {
		UV.fetchSample(sample, 0);
		/*
		ticks++;

		if (sample[0] <= 0.15 && EnExTee.isObstructed == false) {
			EnExTee.isObstructed = true;
			EnExTee.interrupt();
		}
		else { 
			EnExTee.isObstructed = false; 
			EnExTee.restart();
		}

		if (ticks % 100 == 0) {
			LCD.drawString("Ticks: " + ticks, 0, 0); 
		}

		if (ticks % 500 == 0) {
			LCD.drawString("Value: " + sample[0], 0, 1); 
		}
		
	}

} */
