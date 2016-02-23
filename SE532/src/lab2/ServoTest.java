package lab2;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class ServoTest {
	private static RegulatedMotor m;

	public static void main(String[] args) {
		Port port = LocalEV3.get().getPort(MotorPort.A.getName());
		
		m = new EV3LargeRegulatedMotor(port);
		m.setSpeed(700);
		m.forward();
		LCD.drawString("Driving...", 0, 0);

		Delay.msDelay(5000);
		
		m.stop();
		int prestop = m.getTachoCount();
		
		Delay.msDelay(1500);
		int poststop = m.getTachoCount();
		
		int distance = poststop - prestop;
		
		LCD.drawString("Pre-stop: " + prestop, 0, 0);
		LCD.drawString("Post-stop: " + poststop, 0, 1);
		LCD.drawString("Distance: " + distance, 0, 2);
		
		Delay.msDelay(5000);
	}

}
