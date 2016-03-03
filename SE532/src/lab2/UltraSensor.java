package lab2;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class UltraSensor {
	private static SensorModes sensor;

	public static void main(String[] args) {
		Port port = LocalEV3.get().getPort("S1");
		sensor = new EV3UltrasonicSensor(port);
		SampleProvider color = sensor.getMode("Distance");
		float[] sample = new float[color.sampleSize()];
		int iterate = 0;
		
		while (iterate <= 10) {
			color.fetchSample(sample, 0);
			LCD.drawString("Value: " + sample[0], 0, iterate); 
			Delay.msDelay(3000);
			iterate++;
		}
	}
}
