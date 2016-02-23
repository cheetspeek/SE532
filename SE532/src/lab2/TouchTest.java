package lab2;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.*;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class TouchTest {
	public static void main(String[] args) {
		Port port = LocalEV3.get().getPort("S1");
		SensorModes sensor = new EV3TouchSensor(port);
		SampleProvider touched = sensor.getMode("Touch");
		float[] sample = new float[touched.sampleSize()];
		touched.fetchSample(sample, 0);
		
		if (sample[0] == 1) { 
			LCD.drawString("Pressed.", 0, 0); 
		}
		else { LCD.drawString("Not pressed.", 0, 0); }

		Delay.msDelay(3000);
	}

}
