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
		
		newTimer listener = new newTimer(port2, m);
		Timer timer = new Timer(1, listener);
		
		timer.start();
		m.setPower(45);
		m.forward();
		LCD.drawString("Running.", 0, 0);
		Button.DOWN.waitForPressAndRelease();
		m.close();
		
	}
}

class newTimer implements TimerListener {
	
	Port port;
	SampleProvider color;
	float[] sample;
	UnregulatedMotor m;
	
	float white = (float) 0.75;
	float black = (float) 0.2;
	
	Boolean foundBlack = false;
	Boolean isFiveSec = false;
	
	int transitions = 0;
	double RPMcalc = 0.0;
	
	public newTimer(Port port, UnregulatedMotor m) {
		this.port = port;
		this.m = m;
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
		RPMcalc = (transitions / 8) * 12;
		
		if (ticks % 500 == 0) {
			LCD.drawString("RPMs: " + RPMcalc, 1, 1);
			transitions = 0;
		}
		
		if (Math.abs(currentColor - black) <= 0.2) {
			if (foundBlack == false) {
				foundBlack = true;
				transitions++;
				Sound.beep();
			}
		}
		
		else if (Math.abs(currentColor - white) <= 0.2) {
			if (foundBlack == true) {
				foundBlack = false;
				transitions++;
			}
		}
	}
}