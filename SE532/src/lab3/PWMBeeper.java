package lab3;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.utility.Timer;
import lejos.utility.TimerListener;
import lejos.hardware.motor.*;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;

public class PWMBeeper {
	public static void main(String[] args) {
		
		Port port = LocalEV3.get().getPort(MotorPort.A.getName());
		UnregulatedMotor m = new UnregulatedMotor(port);
		
		Port port2 = LocalEV3.get().getPort("S1");
		
		PWMTimer listener = new PWMTimer(port2, m);
		Timer timer = new Timer(1, listener);
		
		timer.start();
		m.setPower(40);
		LCD.drawString("Running.", 0, 0);
		Button.DOWN.waitForPressAndRelease();
		m.close();
		
	}
}

class PWMTimer implements TimerListener {
	
	Port port;
	UnregulatedMotor m;
	
	Boolean isTimeForPulse = false;
	Boolean isFiveSec = false;
	
	int tickTime = 2;
	
	public PWMTimer(Port port, UnregulatedMotor m) {
		this.port = port;
		this.m = m;
	}

	int ticks = 0;
	
	@Override
	public void timedOut() {
		
		ticks++;
		
		isTimeForPulse = (ticks % tickTime == 0);
		isFiveSec = (ticks % 500 == 0);
		
		if (isTimeForPulse) {
			m.forward();
		}
		
		else {
			m.flt(); 
		}
		
		if (ticks % 100 == 0) {
			LCD.drawString("Ticks: " + ticks, 0, 1);
		}
		
		if (isFiveSec) {
			if (tickTime == 2) {
				tickTime = 16;
				LCD.drawString("Slow.", 0, 3);
			}
			else {
				tickTime = 2;
				LCD.drawString("Fast.", 0, 3);
			}
		}
		
	}
}