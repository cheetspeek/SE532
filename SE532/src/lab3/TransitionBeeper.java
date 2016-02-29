package lab3;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class TransitionBeeper {
	public static void main(String[] args) {
		
		Port port = LocalEV3.get().getPort(MotorPort.A.getName());
		UnregulatedMotor m = new UnregulatedMotor(port);
		
		Port port2 = LocalEV3.get().getPort("S1");
		
		newTimer listener = new newTimer(port2, m);
		Timer timer = new Timer(2, listener);
		
		timer.start();
		m.setPower(90);
		LCD.drawString("Running.", 0, 0);
		// m.forward();
		Button.DOWN.waitForPressAndRelease();
		m.close();
		
	}
}

class newTimer implements TimerListener {
	
	Port port;
	SampleProvider color;
	float[] sample;
	UnregulatedMotor m;
	
	float white = (float) 1.0;
	float black = (float) 0.0;
	
	Boolean foundBlack = false;
	
	Boolean isTimeForPulse = false;
	int tickTime = 1;
	
	int transitions = 0;
	int RPMcalc = 0;
	
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
		ticks++;
		
		color.fetchSample(sample, 0);
		currentColor = sample[0];
		
		isTimeForPulse = (ticks % tickTime == 0);
		
		convertedBlackValue = Math.abs(currentColor - black) <= .2;
		convertedWhiteValue = Math.abs(currentColor - white) <= .2;
		
		if (ticks % 600 == 0) {
			if (RPMcalc > 120) {
				tickTime = tickTime + 1;
			} else if (RPMcalc < 120) {
				if (tickTime != 1) { tickTime = tickTime - 1; }
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
		
		if (ticks % 600 == 0) {
			LCD.clear();
			RPMcalc = (transitions / 6) * 37;
			LCD.drawString("RPMs: " + RPMcalc, 0, 1);
			LCD.drawString("Pulses every: " + tickTime, 0, 2);
			LCD.drawString("Transitions: " + transitions, 0, 3);
			LCD.drawString("Ticks: " + ticks, 0, 4);
			transitions = 0;
		}
	}
}