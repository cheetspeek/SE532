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
	
	public static void main(String[] args) {
		Port port = LocalEV3.get().getPort(MotorPort.A.getName());
		RegulatedMotor m1 = new EV3LargeRegulatedMotor(port);
		
		Port port2 = LocalEV3.get().getPort(MotorPort.B.getName());
		RegulatedMotor m2 = new EV3LargeRegulatedMotor(port2);
		
		RegulatedMotor[] list = {m2};
		
		/*
		Port port3 = LocalEV3.get().getPort("S1");
		sensor = new EV3UltrasonicSensor(port3);
		SampleProvider color = sensor.getMode("Distance");
		float[] sample = new float[color.sampleSize()]; */
		
		LCD.drawString("Running.", 0, 0);
		
		int iterations = 0;
		
		m1.setSpeed(300);
		m2.setSpeed(300);
		m1.setAcceleration(165);
		m2.setAcceleration(165);
		
		m1.synchronizeWith(list);
		
		while (iterations <= 8) {
			if (iterations == 8) {
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
	
	public static void rotate(RegulatedMotor m1, RegulatedMotor m2, int speed) {
		m1.startSynchronization();
		m1.rotate(speed, true);
		m2.rotate(Math.abs(speed), true);
		m1.endSynchronization();
		m1.waitComplete();
		m2.waitComplete();
	}
}
