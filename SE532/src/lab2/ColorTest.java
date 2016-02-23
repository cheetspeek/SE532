package lab2;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class ColorTest {
	public static void main(String[] args) {
		Port port = LocalEV3.get().getPort("S1");
		SensorModes sensor = new EV3ColorSensor(port);
		SampleProvider color = sensor.getMode("Red");
		float[] sample = new float[color.sampleSize()];
		int iterate = 0;
		
		while (iterate <= 4) {
			color.fetchSample(sample, 0);
			LCD.drawString("Value: " + sample[0], 0, iterate); 
			Delay.msDelay(3000);
			iterate++;
		}
	}
}
