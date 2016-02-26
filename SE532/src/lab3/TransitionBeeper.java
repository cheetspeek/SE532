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
import lejos.robotics.filter.MeanFilter;

public class TransitionBeeper {
	public static void main(String[] args) {
		
		Port port = LocalEV3.get().getPort(MotorPort.A.getName());
		UnregulatedMotor m = new UnregulatedMotor(port);
		
		Port port2 = LocalEV3.get().getPort("S1");
		
		newTimer listener = new newTimer(port2, m);
		Timer timer = new Timer(1, listener);
		
		timer.start();
		m.setPower(100);
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
	
	float white = (float) 100.0;
	float black = (float) -10.0;
	
	Boolean foundBlack = false;
	
	Boolean isTimeForPulse = false;
	int tickTime = 4;
	
	int transitions = 0;
	double RPMcalc = 0.0;
	
	public newTimer(Port port, UnregulatedMotor m) {
		this.port = port;
		this.m = m;
		@SuppressWarnings("resource")
		SensorModes sensor = new EV3ColorSensor(port);
		color = sensor.getMode("Red");
		color = new MeanFilter(color, 5);
		this.sample = new float[color.sampleSize()];
	}

	int ticks = 0;
	float currentColor = 0;
	Boolean convertedBlackValue;
	Boolean convertedWhiteValue;
	
	@Override
	public void timedOut() {
		// TODO Auto-generated method stub
		ticks++;
		
		color.fetchSample(sample, 0);
		currentColor = (float) Math.floor(181.82 * sample[0] - 36.364);
		
		isTimeForPulse = (ticks % tickTime == 0);
		
		convertedBlackValue = Math.abs(currentColor - black) <= 20;
		convertedWhiteValue = Math.abs(currentColor - white) <= 20;
		
		if (ticks % 2400 == 0) {
			if (RPMcalc > 120) {
				tickTime++;
			}
			else if (RPMcalc < 120) {
				if (tickTime != 1) { tickTime--; }
			}
		}
		
		if (isTimeForPulse) {
			m.forward();
		} else {
			m.flt(); 
		}
		
		if (convertedBlackValue) {
			if (foundBlack == false) {
				foundBlack = true;
				transitions++;
			}
		}
		
		else if (convertedWhiteValue) {
			if (foundBlack == true) {
				foundBlack = false;
				transitions++;
			}
		}
		
		if (ticks % 2400 == 0) {
			// RPMcalc = (transitions / 8) * 12;
			RPMcalc = ((60 * transitions) / 6) / 5;
			LCD.drawString("RPMs: " + RPMcalc, 0, 1);
			LCD.drawString("Tick time: " + tickTime, 0, 2);
			LCD.drawString("Transitions: " + transitions, 0, 3);
			transitions = 0;
		}
	}
}