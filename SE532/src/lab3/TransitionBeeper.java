package lab3;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.utility.Timer;
import lejos.utility.TimerListener;
import lejos.hardware.motor.*;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class TransitionBeeper {
	public static void main(String[] args) {
		
		Port port = LocalEV3.get().getPort(MotorPort.A.getName());
		UnregulatedMotor m = new UnregulatedMotor(port);
		
		Port port2 = LocalEV3.get().getPort("S1");
		
		newTimer listener = new newTimer(port2);
		Timer timer = new Timer(1, listener);
		
		timer.start();
		m.setPower(30);
		m.forward();
		Button.DOWN.waitForPressAndRelease();
		m.close();
		
	}
}

class newTimer implements TimerListener {
	
	Port port;
	SampleProvider color;
	float[] sample;
	
	float white = (float) 0.75;
	float black = (float) 0.2;
	
	Boolean foundBlack = false;
	
	int transitions = 0;
	
	public newTimer(Port port) {
		this.port = port;
		@SuppressWarnings("resource")
		SensorModes sensor = new EV3ColorSensor(port);
		color = sensor.getMode("Red");
		this.sample = new float[color.sampleSize()];
	}

	int ticks = 0;
	float currentColor = 0;
	
	@Override
	public void timedOut() {
		// TODO Auto-generated method stub
		ticks++;
		color.fetchSample(sample, 0);
		currentColor = sample[0];
		
		if (ticks % 1000 == 0) {
			LCD.drawString("Color: " + currentColor, 0, 0);
		}
		
		if (ticks % 5000 == 0) {
			LCD.drawString("Transitions: " + transitions, 0, 1);
			transitions = 0;
		}
		
		if (Math.abs(currentColor - black) <= 0.2) {
			if (foundBlack == false) {
				foundBlack = true;
				transitions++;
				Sound.beep();
			}
			
			// LCD.drawString("Black found.", 0, 1);
		}
		
		else if (Math.abs(currentColor - white) <= 0.2) {
			if (foundBlack == true) {
				foundBlack = false;
				transitions++;
			}
			
			// LCD.drawString("White found.", 0, 1);
		}
		
	}
}