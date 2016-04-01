package lab6;

import java.util.ArrayList;
import java.util.Collections;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class PDController {
	
	static boolean setPrevious = false;
	boolean error50 = false;
	static boolean firstSample = false;
	boolean movingLeft = false;
	public static ArrayList<Integer> sensors = new ArrayList<>();
	double count = 0;
	boolean leftSeenWhite = false;
	boolean rightSeenWhite = false;
	float max, maxPercent, maxLow, maxHigh, min, minPercent, minLow, minHigh;
	static float outerLeftMax = 0;
	static float outerLeftMin = 0;
	static float innerLeftMax = 0;
	static float innerLeftMin = 0;
	static float innerRightMax = 0;
	static float innerRightMin = 0;
	static float outerRightMax = 0;
	static float outerRightMin = 0;
	private int rightModVal = 8;
	private int leftModVal = 8;
	private static int error = 0;
	private static int previousError = 0;
	private static int outerLeftSensorPrev = 0, innerLeftSensorPrev = 0, innerRightSensorPrev = 0, outerRightSensorPrev = 0;
	static int correction = 0;
	static double coefficientP = -.35;
	static double coefficientD = -.15;
	static int speed = 87;

	//For motor
	public static UnregulatedMotor leftServo = new UnregulatedMotor(MotorPort.D);
	public static UnregulatedMotor rightServo = new UnregulatedMotor(MotorPort.A);

	//For color sensor
	private static Port port4 = LocalEV3.get().getPort("S4");
	private static Port port3 = LocalEV3.get().getPort("S3");
	private static Port port2 = LocalEV3.get().getPort("S2");
	private static Port port1 = LocalEV3.get().getPort("S1");

	private static SensorModes outerLeftSensor = new EV3ColorSensor(port4);
	private static SensorModes innerLeftSensor = new EV3ColorSensor(port3);
	private static SensorModes innerRightSensor = new EV3ColorSensor(port2);
	private static SensorModes outerRightSensor = new EV3ColorSensor(port1);

	private static SampleProvider outerLeftSensorProvider = outerLeftSensor.getMode("Red");
	private static SampleProvider innerLeftSensorProvider = innerLeftSensor.getMode("Red");
	private static SampleProvider outerRightSensorProvider = outerRightSensor.getMode("Red");
	private static SampleProvider innerRightSensorProvider = innerRightSensor.getMode("Red");

	private static float[] outerLeftColorSample = new float[outerLeftSensorProvider.sampleSize()];
	private static float[] innerLeftColorSample = new float[innerLeftSensorProvider.sampleSize()];
	private static float[] outerRightColorSample = new float[outerRightSensorProvider.sampleSize()];
	private static float[] innerRightColorSample = new float[innerRightSensorProvider.sampleSize()];
	

	public static void main(String[] args) {
		
		initialCalibration();
		//initializeMotors();
		for(int i = 0; i < 4; i++) {
			sensors.add(-1);
		}
		
		
		while (! Button.DOWN.isDown()) {
			outerRightSensorProvider.fetchSample(outerRightColorSample,0);
			innerLeftSensorProvider.fetchSample(innerLeftColorSample, 0);
			innerRightSensorProvider.fetchSample(innerRightColorSample, 0);
			outerLeftSensorProvider.fetchSample(outerLeftColorSample,0);


			if(!setPrevious) {
				outerRightSensorPrev = normalize(outerRightColorSample[0], "OR");
				innerRightSensorPrev = normalize(innerRightColorSample[0], "IR");
				innerLeftSensorPrev = normalize(innerLeftColorSample[0], "IL");
				outerLeftSensorPrev = normalize(outerLeftColorSample[0], "OL");
				setPrevious = true;
			}

//			int outerLeftSensorSample = normalize(outerLeftColorSample[0], "OL"));
//			int outerRightSensorSample = normalize(outerRightColorSample[0], "OR");
//			int innerRightSensorSample = normalize(innerRightColorSample[0], "IR");
//			int innerLeftSensorSample = normalize(innerLeftColorSample[0], "IL");

			
			sensors.set(0, normalize(outerRightColorSample[0], "OR"));
			sensors.set(1, normalize(innerRightColorSample[0], "IR"));
			sensors.set(2, normalize(innerLeftColorSample[0], "IL"));
			sensors.set(3, normalize(outerLeftColorSample[0], "OL"));

			//if error if negative robot moving to left, if positive robot moving to right
			if(setPrevious) {
				previousError = error;
				error = calculateError();
			}
			
			correction = (int)(coefficientP * error) + (int)(coefficientD * (error - previousError));
			LCD.drawString("Error: " + Integer.toString(error), 0, 3);
			Delay.msDelay(10);
			LCD.clear();
			
			//moving straight 
			if(error >= -40 && error <= 40) {
				rightServo.setPower(speed);
				leftServo.setPower(speed);
			}
			//moving left, correct to right
			else if(error < -40 && error >= -100) {
				rightServo.setPower(Math.abs(correction));
				leftServo.setPower(speed + Math.abs(correction));
			}
			//moving right, correct to left
			else if(error > 40 && error <= 100) {
				rightServo.setPower(speed + Math.abs(correction));
				leftServo.setPower(Math.abs(correction));
			}
		
			Delay.msDelay(500);
			LCD.clear();
		}

	}
	
	private static int calculateError() {
		int error = 0;
		int[] sensorPair = determineSensorPair();
		
		if (sensorPair[0] == 0 || sensorPair[1] == 0) {
			LCD.drawString("S1: " + Integer.toString(sensors.get(0)) + " *", 0, 4);
		}	
		else {
			LCD.drawString("S1: " + Integer.toString(sensors.get(0)), 0, 4);
		}
		
		if (sensorPair[0] == 1 || sensorPair[1] == 1) {
			LCD.drawString("S2: " + Integer.toString(sensors.get(1)) + " *", 0, 5);
		}
		else {
			LCD.drawString("S2: " + Integer.toString(sensors.get(1)), 0, 5);
		}
		
		if (sensorPair[0] == 2 || sensorPair[1] == 2) {
			LCD.drawString("S3: " + Integer.toString(sensors.get(2)) + " *", 0, 6);
		}
		else {
			LCD.drawString("S3: " + Integer.toString(sensors.get(2)), 0, 6);
		}
		
		if (sensorPair[0] == 3 || sensorPair[1] == 3) {
			LCD.drawString("S4: " + Integer.toString(sensors.get(3)) + " *", 0, 7);
		}
		else {
			LCD.drawString("S4: " + Integer.toString(sensors.get(3)), 0, 7);
		}
		
		if ((sensorPair[0] == 0 && sensorPair[1] == 1) || (sensorPair[0] == 1 && sensorPair[1] == 0)) {
			error = (int) -(((sensors.get(0) - sensors.get(1)) * (-.5)) + 68);
			//LCD.drawString("I1", 0, 2);
		}
		else if((sensorPair[0] == 1 && sensorPair[1] == 2) || (sensorPair[0] == 2 && sensorPair[1] == 1)) {
			error = (int) ((sensors.get(1) - sensors.get(2)) * .3);
			//LCD.drawString("I2", 0, 2);
		}
		else if ((sensorPair[0] == 2 && sensorPair[1] == 3) || (sensorPair[0] == 3 && sensorPair[1] == 2)) {
			error = (int) (((sensors.get(3) - sensors.get(2)) * (-.5)) + 68) ;
			//LCD.drawString("I3", 0, 2);
		}
	
		return (error > 100) ? 100 : (error < -100) ? -100 : error;
	}
	
	public static int[] determineSensorPair() {
		int[] sensorPair = new int[2];
		ArrayList<Integer> tempSensors = new ArrayList<>();
		for(int p : sensors) {
		    tempSensors.add(p);
		}
		
		int firstMinIndex = getMin(tempSensors, true);
		sensorPair[0] = firstMinIndex;
		tempSensors.set(firstMinIndex, -1);
		int secondMinIndex = getMin(tempSensors, false);
		sensorPair[1] = secondMinIndex;	
		
		//LCD.drawString("S" + Integer.toString(firstMinIndex), 0, 0);
		//LCD.drawString("S" + Integer.toString(secondMinIndex), 0, 1);
		
		return sensorPair;		
	}
	
	public static int getMin(ArrayList<Integer> list, boolean initPass) {
		int index = -1;
		if (initPass) {
			index = list.indexOf(Collections.min(list));
		}
		else {
			if(list.get(1) == -1 && list.get(2) < 97) {
				index = 2;
			}
			else if(list.get(2) == -1 && list.get(1) < 97) {
				index = 1;
			}
			else {
				if(list.get(0) == -1)
					index = 1;
				else if(list.get(1) == -1)
					index = 0;
				else if(list.get(2) == -1)
					index = 3;
				else
					index = 2;
			}
		}
		return index;
	}

	private static void initializeMotors() {
		leftServo.setPower(100);
		rightServo.setPower(100);
	}

	public static void initialCalibration() {
		float[] outerLeftSamples = new float[100];
		float[] innerLeftSamples = new float[100];
		float[] innerRightSamples = new float[100];
		float[] outerRightSamples = new float[100];


		while(!Button.UP.isDown()) {
			LCD.drawString("Press to cali Black", 0, 0);
		}

		//get max color and min color
		for(int i = 0; i < 50; i++) {
			innerLeftSensorProvider.fetchSample(innerLeftColorSample, 0);
			innerRightSensorProvider.fetchSample(innerRightColorSample, 0);
			outerLeftSensorProvider.fetchSample(outerLeftColorSample, 0);
			outerRightSensorProvider.fetchSample(outerRightColorSample, 0);
			Delay.msDelay(10);
			outerLeftSamples[i] = outerLeftColorSample[0];
			innerLeftSamples[i] = innerRightColorSample[0];
			innerRightSamples[i] = innerRightColorSample[0];
			outerRightSamples[i] = outerRightColorSample[0];
		}

		while(!Button.UP.isDown()) {
			LCD.drawString("Press to cali White", 0, 0);
			LCD.drawString("BLACK: " + Float.toString(outerLeftSamples[0]), 0, 2);
		}

		for(int i = 50; i < 100; i++) {
			innerLeftSensorProvider.fetchSample(innerLeftColorSample, 0);
			innerRightSensorProvider.fetchSample(innerRightColorSample, 0);
			outerLeftSensorProvider.fetchSample(outerLeftColorSample, 0);
			outerRightSensorProvider.fetchSample(outerRightColorSample, 0);
			Delay.msDelay(10);
			outerLeftSamples[i] = outerLeftColorSample[0];
			innerLeftSamples[i] = innerRightColorSample[0];
			innerRightSamples[i] = innerRightColorSample[0];
			outerRightSamples[i] = outerRightColorSample[0];
		}

		outerLeftMax = calculateMaxMin(outerLeftSamples, true);
		outerLeftMin = calculateMaxMin(outerLeftSamples, false);

		innerLeftMax = calculateMaxMin(innerLeftSamples, true);
		innerLeftMin = calculateMaxMin(innerLeftSamples, false);

		innerRightMax = calculateMaxMin(innerRightSamples, true);
		innerRightMin = calculateMaxMin(innerRightSamples, false);

		outerRightMax = calculateMaxMin(outerRightSamples, true);
		outerRightMin = calculateMaxMin(outerRightSamples, false);

		firstSample = true;
	}

	public static float calculateMaxMin(float[] samples, boolean getMax) {
		float value = samples[0];

		for (int i = 1; i < samples.length; i++) {
			if (samples[i] > value && getMax) {
				value = samples[i];
			}
			else if(samples[i] < value && !getMax) {
				value = samples[i];
			}
		}

		return value;
	}

	private static int normalize(float reading, String sensor) {
		int result;

		if(sensor.equals("OL")) {
			result = (int) Math.abs((((reading - outerLeftMin) / (outerLeftMax - outerLeftMin)) * 100));
		}
		else if (sensor.equals("IL")) {
			result = (int) Math.abs((((reading - innerLeftMin) / (innerLeftMax - innerLeftMin)) * 100));
		}
		else if (sensor.equals("IR")) {
			result = (int) Math.abs((((reading - innerRightMin) / (innerRightMax - innerRightMin)) * 100));
		}
		else { //if(sensor.equals("OR"))
			result = (int) Math.abs((((reading - outerRightMin) / (outerRightMax - outerRightMin)) * 100));
		}
		
		if(result > 100) {
			result = 100;
		}

		return result;
	}
}
