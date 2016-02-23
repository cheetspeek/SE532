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

public class PWMBeeper {
	public static void main(String[] args) {
		
		Port port = LocalEV3.get().getPort(MotorPort.A.getName());
		UnregulatedMotor m = new UnregulatedMotor(port);
		
		Port port2 = LocalEV3.get().getPort("S1");
		
		PWMTimer listener = new PWMTimer(port2, m);
		Timer timer = new Timer(2, listener);
		
		timer.start();
		m.setPower(40);
		LCD.drawString("Running.", 0, 0);
		Button.DOWN.waitForPressAndRelease();
		m.close();
		
	}
}

class PWMTimer implements TimerListener {
	
	Port port;
	SampleProvider color;
	float[] sample;
	UnregulatedMotor m;
	
	float white = (float) 0.75;
	float black = (float) 0.2;
	
	Boolean foundBlack = false;
	
	Boolean isTimeForPulse = false;
	Boolean isFiveSec = false;
	
	int transitions = 0;
	int RPMcalc = 0;
	int tickTime = 2;
	
	public PWMTimer(Port port, UnregulatedMotor m) {
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
		
		ticks++;
		color.fetchSample(sample, 0);
		currentColor = sample[0];
		RPMcalc = (transitions / 8) * 12;
		
		isTimeForPulse = (ticks % tickTime == 0);
		isFiveSec = (ticks % 500 == 0);
		
		if (isTimeForPulse) {
			m.forward();
		}
		
		else { m.flt(); }
		
		if (ticks % 100 == 0) {
			LCD.drawString("Ticks: " + ticks, 0, 1);
		}
		
		if (isFiveSec) {
			LCD.drawString("RPMs: " + RPMcalc, 1, 2);
			transitions = 0;
			if (tickTime == 2) {
				tickTime = 16;
				LCD.drawString("Slow.", 0, 3);
			}
			else {
				tickTime = 2;
				LCD.drawString("Fast.", 0, 3);
			}
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