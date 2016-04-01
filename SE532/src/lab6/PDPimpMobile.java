package lab6;

import java.util.ArrayList;
import java.util.Collections;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class PDPimpMobile{
	
	static int MOTOR_SPEED = 90;
	static double P_COEFF = 0.699;
	static double D_COEFF = 0.019;
	
	static float whiteOne = 0;
	static float whiteTwo = 0;
	static float whiteThree = 0;
	static float whiteFour = 0;
	
	static float blackOne = 0;
	static float blackTwo = 0;
	static float blackThree = 0;
	static float blackFour = 0;
	
	static UnregulatedMotor leftMotor = null; 
	static UnregulatedMotor rightMotor = null;
	
	static SensorModes sensorOne = null;
	static SensorModes sensorTwo = null;
	static SensorModes sensorThree = null;
	static SensorModes sensorFour = null;
	
	static SampleProvider sensorProviderOne = null;
	static SampleProvider sensorProviderTwo = null;
	static SampleProvider sensorProviderThree = null;
	static SampleProvider sensorProviderFour = null;
	
	static float[] sampleOne = null;
	static float[] sampleTwo = null;
	static float[] sampleThree = null;
	static float[] sampleFour = null;
	
	static float[] slopes = new float[4];
	static float[] intercepts = new float[4];
	
	static ArrayList<Integer> samples = new ArrayList<Integer>();
	static ArrayList<Integer> minimumPair = new ArrayList<Integer>();
	
	static int error = 0;
	static int lastError = 0;
	static int correction = 0;
	
	
	public static void main (String[] args){
		initialize();
		
		calibrateColors();
				
		Delay.msDelay(2000);
		LCD.clear();
		
		startFollowingLine();
		//displayErrorRange();
	}
	
	public static void displayErrorRange(){
		
		while(Button.UP.isUp()){
			
			
			fetchAllSamples();
			minimumSensorPair();
			
			error = errorCalculation();
			
			for(int i = 1; i <= 4; i++){
				if(minimumPair.contains(i)){
					LCD.drawString("S" + i + ": " + samples.get(i - 1) + "*" , 0, i);
				}else{
					LCD.drawString("S" + i + ": " + samples.get(i - 1), 0, i);
				}
			}
			
			int error = errorCalculation();
			LCD.drawString("E: " + error, 0, 5);
			
			Delay.msDelay(500);
			LCD.clear();
		}
	}
	
	public static void startFollowingLine(){
		setSpeedAndCoeff();
		
		leftMotor.backward();
		rightMotor.backward();
		
		LCD.clear();
		LCD.drawString("PRESS ENTER TO GO", 0, 0);
		Button.ENTER.waitForPressAndRelease();
		
		while(Button.UP.isUp()){
			fetchAllSamples();
			minimumSensorPair();
		
			LCD.clear();
			
			error = errorCalculation();
			
			correction = (int)(P_COEFF * error) + (int) (D_COEFF * lastError);
			
			if(error < 0){
				// turn left
				rightMotor.setPower(MOTOR_SPEED - Math.abs(correction)); 
				leftMotor.setPower(MOTOR_SPEED + 5);
			}else if(error > 0){
				// turn right
				leftMotor.setPower(MOTOR_SPEED - Math.abs(correction)); 
				rightMotor.setPower(MOTOR_SPEED + 5);
			}else{
				leftMotor.setPower(MOTOR_SPEED);
				rightMotor.setPower(MOTOR_SPEED);
			}
			
			lastError = error;
		
		}
		Delay.msDelay(1000);
		LCD.clear();
		leftMotor.stop();
		rightMotor.stop();
		LCD.drawString("Enter to Cont.", 0, 0);
		LCD.drawString("Other to Quit", 0, 1);
		int id = Button.waitForAnyPress();
		if(id == Button.ID_ENTER){
			LCD.clear();
			startFollowingLine();
		}
		
	}
	
	public static int errorCalculation(){
		int e = 0;
		
		if(minimumPair.contains(1) && minimumPair.contains(2)){ // if sensor 1 and 2 are minimum
			e = (int) -(((samples.get(0) - samples.get(1)) * (-.5)) + 68);
		}else if(minimumPair.contains(2) && minimumPair.contains(3)){
			e = (int) ((samples.get(1) - samples.get(2)) * 0.3);
		}else if(minimumPair.contains(3) && minimumPair.contains(4)){
			e = (int) (((samples.get(3) - samples.get(2)) * (-.5)) + 68);
		}
		
		return (e > 100) ? 100 : (e < -100) ? -100 : e;
	}
	
	private static void setSpeedAndCoeff(){
		boolean speed = false;
		boolean p_coeff = false;
		boolean d_coeff = false;
		
		while(!speed || !p_coeff || !d_coeff){
			
			if(!speed){
				LCD.drawString("SPEED: " + MOTOR_SPEED, 0, 0);
				int id = Button.waitForAnyPress();
				if(id == Button.ID_UP){
					MOTOR_SPEED += 5;
				}
				
				if(id == Button.ID_DOWN){
					MOTOR_SPEED -= 5;
				}
				
				if(id == Button.ID_RIGHT){
					MOTOR_SPEED += 1;
				}
				
				if(id == Button.ID_LEFT){
					MOTOR_SPEED -= 1;
				}
				
				if(id == Button.ID_ENTER){
					speed = true;
				}
				
				LCD.drawString("SPEED: " + MOTOR_SPEED, 0, 0);
				
			}else if(!p_coeff){
				LCD.drawString("P_COEFF: " + P_COEFF, 0, 0);
				int id = Button.waitForAnyPress();
				if(id == Button.ID_UP){
					P_COEFF += 0.1;
				}
				
				if(id == Button.ID_DOWN){
					P_COEFF -= 0.1;
				}
				
				if(id == Button.ID_RIGHT){
					P_COEFF += .01;
				}
				
				if(id == Button.ID_LEFT){
					P_COEFF -= .01;
				}
				
				if(id == Button.ID_ENTER){
					p_coeff = true;
				}
				
				LCD.drawString("P_COEFF: " + P_COEFF, 0, 0);
				
			}else if(!d_coeff){
				LCD.drawString("D_COEFF: " + D_COEFF, 0, 0);
				int id = Button.waitForAnyPress();
				if(id == Button.ID_UP){
					D_COEFF += 0.1;
				}
				
				if(id == Button.ID_DOWN){
					D_COEFF -= 0.1;
				}
				
				if(id == Button.ID_RIGHT){
					D_COEFF += .01;
				}
				
				if(id == Button.ID_LEFT){
					D_COEFF -= .01;
				}
				
				if(id == Button.ID_ENTER){
					d_coeff = true;
				}
				
				LCD.drawString("D_COEFF: " + D_COEFF, 0, 0);
			}
		}
		
	}
	
	private static void initialize(){
		leftMotor = new UnregulatedMotor(MotorPort.D);
		rightMotor = new UnregulatedMotor(MotorPort.A);
						
		sensorOne = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
		sensorTwo = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
		sensorThree = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
		sensorFour = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
		
		sensorProviderOne = sensorOne.getMode("Red");
		sensorProviderTwo = sensorTwo.getMode("Red");
		sensorProviderThree = sensorThree.getMode("Red");
		sensorProviderFour = sensorFour.getMode("Red");
		
		sampleOne = new float[sensorProviderOne.sampleSize()];
		sampleTwo = new float[sensorProviderTwo.sampleSize()];
		sampleThree = new float[sensorProviderThree.sampleSize()];
		sampleFour = new float[sensorProviderFour.sampleSize()];
		
		// initialize array so we can use samples.set() in fetchAllSamples
		samples.add(-1);
		samples.add(-1);
		samples.add(-1);
		samples.add(-1);
		
		minimumPair.add(-1);
		minimumPair.add(-1);
	
	}
	
	private static void calibrateColors(){
		LCD.drawString("Set White: ", 0, 0);
		Button.UP.waitForPressAndRelease();
		sensorProviderOne.fetchSample(sampleOne, 0);
		sensorProviderTwo.fetchSample(sampleTwo, 0);
		sensorProviderThree.fetchSample(sampleThree, 0);
		sensorProviderFour.fetchSample(sampleFour, 0);
		
		whiteOne = sampleOne[0];
		whiteTwo = sampleTwo[0];
		whiteThree = sampleThree[0];
		whiteFour = sampleFour[0];
		LCD.clear();
		
		LCD.drawString("Set Black: ", 0, 0);
		Button.UP.waitForPressAndRelease();
		sensorProviderOne.fetchSample(sampleOne, 0);
		sensorProviderTwo.fetchSample(sampleTwo, 0);
		sensorProviderThree.fetchSample(sampleThree, 0);
		sensorProviderFour.fetchSample(sampleFour, 0);
		
		blackOne = sampleOne[0];
		blackTwo = sampleTwo[0];
		blackThree = sampleThree[0];
		blackFour = sampleFour[0];
		LCD.clear();
		
		slopes[0] = findSlope(blackOne, whiteOne);
		slopes[1] = findSlope(blackTwo, whiteTwo);
		slopes[2] = findSlope(blackThree, whiteThree);
		slopes[3] = findSlope(blackFour, whiteFour);
		
		intercepts[0] = findIntercept(slopes[0], whiteOne);
		intercepts[1] = findIntercept(slopes[1], whiteTwo);
		intercepts[2] = findIntercept(slopes[2], whiteThree);
		intercepts[3] = findIntercept(slopes[3], whiteFour);
		
		LCD.drawString("1W:" + whiteOne + "  1B:" + blackOne, 0, 0);
		LCD.drawString("2W:" + whiteTwo + "  2B:" + blackTwo, 0, 1);
		LCD.drawString("3W:" + whiteThree + "  3B:" + blackThree, 0, 2);
		LCD.drawString("4W:" + whiteFour + "  4B:" + blackFour, 0, 3);
		
	}
	
	
	private static float findSlope(float x1, float x2){
		return 100/(x2 - x1);
	}
	
	private static float findIntercept(float slope, float x2){
		return 100 - (slope * x2);
	}
	
	private static void fetchAllSamples(){
		samples.set(0, fetchSample('1'));
		samples.set(1, fetchSample('2'));
		samples.set(2, fetchSample('3'));
		samples.set(3, fetchSample('4'));
	}
	
	private static int fetchSample(char sensor){
		int rtnVal = 0;
		switch (sensor){
			case '1': 
				sensorProviderOne.fetchSample(sampleOne, 0);
				rtnVal = (int) Math.floor((sampleOne[0] * slopes[0]) + intercepts[0]);
				break;
			case '2': 
				sensorProviderTwo.fetchSample(sampleTwo, 0);
				rtnVal = (int) Math.floor((sampleTwo[0] * slopes[1]) + intercepts[1]);
				break;
			case '3': 
				sensorProviderThree.fetchSample(sampleThree, 0);
				rtnVal = (int) Math.floor((sampleThree[0] * slopes[2]) + intercepts[2]);
				break;
			case '4': 
				sensorProviderFour.fetchSample(sampleFour, 0);
				rtnVal = (int) Math.floor((sampleFour[0] * slopes[3]) + intercepts[3]);
				break;
		}
		
		if(rtnVal > 100) { rtnVal = 100;}
		if(rtnVal < 0) { rtnVal = 0;}
		return rtnVal;
	}
	
	
	private static void minimumSensorPair(){
		int firstMinimum = getMinimumSensor(true, -1);
		int secondMinimum = getMinimumSensor(false, firstMinimum);
		
		minimumPair.set(0, firstMinimum);
		minimumPair.set(1, secondMinimum);
	}
	
	private static int getMinimumSensor(boolean firstMin, int firstIndex){
		int minSensor = 0; // actual sensor, does not start from 0
		if(firstMin){
			minSensor = samples.indexOf(Collections.min(samples));
			minSensor++;
		}else{
			if(firstIndex == 2 && samples.get(2) < 70){
				minSensor = 3;
			}else if(firstIndex == 3 && samples.get(1) < 70){
				minSensor = 2;
			}else{
				if(firstIndex == 1){
					minSensor = 2;
				}else if(firstIndex == 2){
					minSensor = 1;
				}else if(firstIndex == 3){
					minSensor = 4;
				}else if(firstIndex == 4){
					minSensor = 3;
				}
			}
		}
		
		return minSensor;
	}
	
	
	public static int middleErrorCalc(int left, int right){
		return (int) Math.floor((left - right) * 0.33);
	}
	
	public static int outerErrorCalculation(int outer, int inner){
		return (int) Math.floor(((outer - inner) * -0.5) + 68);
	}
	

}
