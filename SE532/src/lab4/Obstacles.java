package lab4;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class Obstacles implements TimerListener {
	private static EV3UltrasonicSensor ultrasonicSensor;
	private static Port motorPort1;
	private static Port motorPort2;
	private static Port sensorPort;
	public static RegulatedMotor motor1;
	private static RegulatedMotor motor2;
	private static SampleProvider sampleProvider;
	private static float[] sample;
	private static Timer timer;
	protected static int angle = 0;
	protected static float distance = 0;
	public static int turnAngleCounter = 0;

	public static void main(String[] args) {
		setup();

		int iterations = 0;

		timer.start();

		// loop guard and if must be the same
		while (iterations <= 32) {
			// Sends GoBot to final block
			if (iterations == 32) {
				angle = 1690;
				rotate(motor1,motor2,angle);
				iterations++;
			}
			// Sends GoBot to next block
			else if (iterations % 2 == 0) {
				angle = 780;
				rotate(motor1,motor2,angle);
				iterations++;
			}
			// Turns GoBot
			else {
				if (turnAngleCounter % 2 == 0) {
					angle = -165;
				} else {
					angle = -166;
				}
				turnAngleCounter++;
				rotate(motor1,motor2,angle);
				iterations++;
			}
		}

		timer.stop();

		motor1.close();
		motor2.close();
	}

	public static void setup() {
		motorPort1 = LocalEV3.get().getPort(MotorPort.A.getName());
		motor1 = new EV3LargeRegulatedMotor(motorPort1);

		motorPort2 = LocalEV3.get().getPort(MotorPort.B.getName());
		motor2 = new EV3LargeRegulatedMotor(motorPort2);

		RegulatedMotor[] list = {motor2};

		sensorPort = LocalEV3.get().getPort("S1");
		ultrasonicSensor = new EV3UltrasonicSensor(sensorPort);

		sampleProvider = ultrasonicSensor.getMode("Distance");
		sample = new float[sampleProvider.sampleSize()];

		timer = new Timer(1, new Obstacles());

		LCD.drawString("Running.", 0, 0);

		motor1.synchronizeWith(list);

		motor1.setSpeed(400);
		motor2.setSpeed(400);
		motor1.setAcceleration(150);
		motor2.setAcceleration(150);
	}

	public static void rotate(RegulatedMotor m1, RegulatedMotor m2, int angle) {
		m1.startSynchronization();
		m1.rotate(angle);
		m2.rotate(Math.abs(angle));
		m1.endSynchronization();
		m1.waitComplete();
		m2.waitComplete();
	}

	public static void setMotorSpeedSlow(RegulatedMotor m1, RegulatedMotor m2) {
		motor1.setSpeed(1);
		motor2.setSpeed(1);
	}

	public static void setMotorSpeedFast(RegulatedMotor m1, RegulatedMotor m2) {
		motor1.setSpeed(400);
		motor2.setSpeed(400);
	}

	@Override
	public void timedOut() {
		sampleProvider.fetchSample(sample, 0);

		LCD.drawString("Distance: " + sample[0], 0, 0);

		if (sample[0] < 0.3) {
			setMotorSpeedSlow(motor1,motor2);
			LCD.drawString("Slow down.", 0, 3);
		} else {
			setMotorSpeedFast(motor1,motor2);
			LCD.drawString("Speed up.", 0, 3);
		}
	}
}