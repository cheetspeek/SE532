package lab4;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class EnExTee {
	private static SensorModes sensor;
	private static Port port;
	private static Port port2;
	private static Port port3;
	private static float[] sample;
	private static RegulatedMotor m1;
	private static RegulatedMotor m2;
	
	public static void main(String[] args) {
		setup();
		
		int iterations = 0;
		
		while (iterations <= 2) {
			if (iterations == 6) {
				rotate(m1,m2,1600);
				iterations++;
			}
			else if (iterations % 2 == 0) {
				rotate(m1,m2,780);
				iterations++;
			}
			else {
				rotate(m1,m2,-165);
				iterations++;
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
		SampleProvider color = sensor.getMode("Distance");
		sample = new float[color.sampleSize()];
		
		LCD.drawString("Running.", 0, 0);
		
		m1.setSpeed(300);
		m2.setSpeed(300);
		m1.setAcceleration(165);
		m2.setAcceleration(165);
		
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
}
