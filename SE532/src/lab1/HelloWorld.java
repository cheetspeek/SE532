package lab1;

import lejos.hardware.lcd.*;
import lejos.utility.Delay;

public class HelloWorld {
	public static void main(String[] args) {
		LCD.drawString("Hello, world!", 0, 0);
		LCD.drawString("I'm an EV3.", 0, 1);
		Delay.msDelay(3000);
	}

}
