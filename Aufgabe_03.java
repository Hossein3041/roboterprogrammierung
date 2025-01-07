package test;

import java.awt.Color;
import lejos.hardware.Battery;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.Port;

class MeasureDistance{
	private EV3UltrasonicSensor distanceSensor;
	
	public MeasureDistance(Port port) {
		this.distanceSensor = new EV3UltrasonicSensor(port);
	}
	
	public float getDistance() {
		float[] sample = new float[distanceSensor.sampleSize()];
		distanceSensor.getDistanceMode().fetchSample(sample, 0);
		float distance = sample[0];
		return distance;
	}
	
	public void closeDistanceSensor() {
		distanceSensor.close();
	}
}

class MeasureBrightness{
	private EV3ColorSensor colorSensor;
	public MeasureBrightness(EV3ColorSensor sensor) {
		this.colorSensor = sensor;
	}
	public int measureBrightness() {
		SampleProvider colorProvider = colorSensor.getRedMode();
		float[] sample = new float[colorProvider.sampleSize()];
		colorSensor.fetchSample(sample, 0);
		int brightness = (int)(sample[0] * 100);
		return brightness;
	}
	public void closeColorSensor() {
		colorSensor.close();
	}
}

class VehicleController{
	private static EV3LargeRegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.B);
	private static EV3LargeRegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.C);
	private static MeasureDistance measureDistance = new MeasureDistance(SensorPort.S4);
	
	private static int startSpeed = 350;
	private static int targetThreshold;
	
	private static double Kp = 0.001;
	private static double Ki = 0.001;
	private static double Kd = 0.001;
	private static double previousError = 0;
	private static double integral = 0;
	private static final float SAFE_DISTANCE = 0.3f;
	
	public static void setThreshold(int target) {
		targetThreshold = target;
	}
	public static void startVehicle(TextLCD lcd) {
		initializeAndStartMotors();
		Thread pidThread = new Thread(() ->{
			while(Button.ENTER.isUp()) {
				pidRegler();
			}
		});
		pidThread.start();
		Button.ENTER.waitForPressAndRelease();
		stopMotors();
	}
	
	private static void initializeAndStartMotors() {
		motorLeft.resetTachoCount();
		motorRight.resetTachoCount();
		motorLeft.setSpeed(startSpeed);
		motorRight.setSpeed(startSpeed);
		motorLeft.forward();
		motorRight.forward();
	}
	
	private static void pidRegler() {
	    MeasureBrightness measure = new MeasureBrightness(DisplayInfo.colorSensor);
	    int currentBrightness = measure.measureBrightness();
	    int error = targetThreshold - currentBrightness;
	    
	    boolean tooClose = measureDistance.getDistance() < SAFE_DISTANCE;

	    int output = calculatePIDoutput(error);

	    if (currentBrightness < targetThreshold) { 
	        stopMotors();
	        searchForLine();
	    } else {
	    	adjustMotors(tooClose, output);
	    }
	    previousError = error;
	}
	private static int calculatePIDoutput(int error) {
		integral += error;
		double derivative = error - previousError;
		int maxIntegral = 100;
		integral = Math.max(Math.min(integral, maxIntegral), -maxIntegral);
		
		double kpFaktor = Kp * error;
		double kiFaktor = Ki * integral;
		double kdFaktor = Kd * derivative;
		
		int output = (int)(kpFaktor + kiFaktor + kdFaktor);
		return output;
	}

	private static void searchForLine() {
	    int leftTurnDuration = 700;
	    int rightTurnDuration = 1500;
		boolean lineFound = false;
	    while(!lineFound) {
	    	if(measureDistance.getDistance() < SAFE_DISTANCE) stopMotors();
	    	turnLeft();
	    	
	    	lineFound = waitForLineDetection(leftTurnDuration / 100);
	    	if(lineFound) break;
	    	
	    	rightTurnDuration += 250;
	    	if(measureDistance.getDistance() < SAFE_DISTANCE) stopMotors();
	    	turnRight();
	    	
	    	lineFound = waitForLineDetection(rightTurnDuration / 100);
	    	if(lineFound) break;
	    	
	    	leftTurnDuration += 250;
	    }
	    stopMotors();
	    pause(50);
    }
	
	private static void turnLeft() {
		motorLeft.setSpeed(50);
		motorRight.setSpeed(50);
		motorLeft.forward();
		motorRight.backward();
	}
	
	private static void turnRight() {
		motorLeft.setSpeed(50);
		motorRight.setSpeed(50);
		motorLeft.backward();
		motorRight.forward();
	}
	private static void pause(int milliseconds) {
		try {
			Thread.sleep(milliseconds);
		} catch(InterruptedException e) {
			Thread.currentThread().interrupt();
		}
	}
	
	private static boolean waitForLineDetection(int intervals) {
		MeasureBrightness measure = new MeasureBrightness(DisplayInfo.colorSensor);
		for(int i = 0; i < intervals; ++i) {
			try {
				Thread.sleep(100);
			} catch(InterruptedException e) {
				Thread.currentThread().interrupt();
				return false;
			}
			
			if(measure.measureBrightness() >= targetThreshold) {
				return true;
			}
		}
		return false;
	}
	
	private static void adjustMotors(boolean adjustDistance, int correction) {
		if(adjustDistance)
			adjustMotorsForDistance();
		else
			adjustMotorsForCorrection(correction);
	}
	
	private static void adjustMotorsForDistance() {
		float distance = measureDistance.getDistance();
		 if (distance < 0.24)
			 setMotorSpeeds(0, 0);
		 else if (distance < 0.27)
			 setMotorSpeeds((int)(startSpeed * 0.25), (int)(startSpeed * 0.25));
		 else if (distance < 0.30)
			 setMotorSpeeds((int)(startSpeed * 0.50), (int)(startSpeed * 0.50));
		 else
			 setMotorSpeeds((int)(startSpeed * 0.75), (int)(startSpeed * 0.75));
	}
	
	private static void adjustMotorsForCorrection(int correction) {
		int newSpeedLeft = startSpeed - correction;
		int newSpeedRight = startSpeed + correction;
		pause(250);
        incrementMotorSpeeds(newSpeedLeft, newSpeedRight);
	}
	
	private static void incrementMotorSpeeds(int targetSpeedLeft, int targetSpeedRight) {
		int currentSpeedLeft = motorLeft.getSpeed();
		int currentSpeedRight = motorRight.getSpeed();
		int increment = 10;
		
		MeasureBrightness measure = new MeasureBrightness(DisplayInfo.colorSensor);
		
		while(currentSpeedLeft != targetSpeedLeft || currentSpeedRight != targetSpeedRight) {
			if(checkLineLost(measure)) return;
			
			if(measureDistance.getDistance() < SAFE_DISTANCE) stopMotors();
			currentSpeedLeft = adjustSpeed(currentSpeedLeft, targetSpeedLeft, increment);
			
			if(checkLineLost(measure)) return;
			
			if(measureDistance.getDistance() < SAFE_DISTANCE) stopMotors();
			currentSpeedRight = adjustSpeed(currentSpeedRight, targetSpeedRight, increment);
			
			if(checkLineLost(measure)) return;
			
			if(measureDistance.getDistance() < SAFE_DISTANCE) stopMotors();
			setMotorSpeeds(currentSpeedLeft, currentSpeedRight);
			pause(50);
		}
	}
	
	private static int adjustSpeed(int currentSpeed, int targetSpeed, int increment) {
		if(currentSpeed < targetSpeed) {
			return Math.min(currentSpeed + increment, targetSpeed);
		} else {
			return Math.max(currentSpeed - increment, targetSpeed);
		}
	}
	
	private static boolean checkLineLost(MeasureBrightness measure) {
		if(measure.measureBrightness() < targetThreshold) {
			stopMotors();
			searchForLine();
			return true;
		}
		return false;
	}
	
	private static void setMotorSpeeds(int leftSpeed, int rightSpeed) {
		motorLeft.setSpeed(leftSpeed);
		motorRight.setSpeed(rightSpeed);
		motorLeft.forward();
		motorRight.forward();
	}

	public static void stopMotors() {
		motorLeft.stop(true);
		motorRight.stop(true);
	}
	
	public static void closeConnection() {
		motorLeft.close();
		motorRight.close();
	}
}


class Calibrate{	
	public static void calibrate(TextLCD lcd) {
		MeasureBrightness measure = new MeasureBrightness(DisplayInfo.colorSensor);
		lcd.drawString("Line measurement", 0, 0);
		Button.waitForAnyPress();
		lcd.clear();
		int lineBrightness = measure.measureBrightness();
		lcd.drawString("Linebright.: " + lineBrightness, 0, 0);
		Button.waitForAnyPress();
		lcd.clear();
		
		lcd.drawString("Floor measurement", 0, 0);
		Button.waitForAnyPress();
		lcd.clear();
		int floorBrightness = measure.measureBrightness();
		lcd.drawString("Floorbright.: " + floorBrightness, 0, 0);
		Button.waitForAnyPress();
		lcd.clear();
		
		int targetThreshold = (lineBrightness + floorBrightness) / 2;
		lcd.drawString("TargetThresh. " + targetThreshold, 0, 0);
		VehicleController.setThreshold(targetThreshold);
		Button.waitForAnyPress();
		lcd.clear();
	}
}

class Controller{
	private TextLCD lcd = LocalEV3.get().getTextLCD();	
	public Controller() {
		start();
	}
	private void start() {
		boolean go = true;
        try {
        	do {
                lcd.drawString("Press ENTER", 0, 0);
                Button.waitForAnyPress();
                lcd.clear();

                lcd.drawString("Start calibrating", 0, 0);
                Button.waitForAnyPress();
                lcd.clear();

                Calibrate.calibrate(lcd);

                lcd.drawString("Finished calibr.", 0, 0);
                Button.waitForAnyPress();
                lcd.clear();

                lcd.drawString("ENTER for Start", 0, 0);
                Button.waitForAnyPress();
                VehicleController.startVehicle(lcd);

                lcd.drawString("ESC to exit", 0, 0);
                int pressedButton = Button.waitForAnyPress();
                lcd.clear();
                if (pressedButton == Button.ID_ESCAPE) {
                	go = false;
                	throw new RuntimeException("0");
                }
            } while (go);
        } catch (RuntimeException e) {
            System.out.println("Program exited: " + e.getMessage());
        }
    }
}

public class Aufgabe_03 {
	public static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);
	public static void main(String[] args) {
		Runtime.getRuntime().addShutdownHook(new Thread(() ->{
			TextLCD lcd = LocalEV3.get().getTextLCD();
			lcd.drawString("Cleanup operation...", 0, 0);
			 VehicleController.stopMotors();
			 VehicleController.closeConnection();
			 MeasureBrightness cSensor = new MeasureBrightness(colorSensor);
			 cSensor.closeColorSensor();
		}));
		new Controller();
	}
}
