package lab2;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class FullTest {
	public static void main(String[] args) {
		Port port = LocalEV3.get().getPort("S1");
		SensorModes sensor = new EV3TouchSensor(port);
		SampleProvider touched = sensor.getMode("Touch");
		float[] sample = new float[touched.sampleSize()];
		
		Port portT = LocalEV3.get().getPort("S2");
		SensorModes sensor2 = new EV3TouchSensor(portT);
		SampleProvider touched2 = sensor2.getMode("Touch");
		float[] sample2 = new float[touched2.sampleSize()];
		
		boolean run = true;
		
		Port port2 = LocalEV3.get().getPort(MotorPort.A.getName());
		RegulatedMotor m = new EV3LargeRegulatedMotor(port2);
		m.setSpeed(700);
		
		while (run) {
			touched.fetchSample(sample, 0);
			touched2.fetchSample(sample2, 0);
			if (sample[0] == 1) { m.forward(); }
			else { m.flt(); }
			if (sample2[0] == 1) { run = false; }
		}
	}
}
