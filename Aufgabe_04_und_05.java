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
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.jfree.data.xy.Vector;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

class MeasureDistance {
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

class MeasureBrightness {
    private EV3ColorSensor colorSensor;

    public MeasureBrightness(EV3ColorSensor sensor) {
        this.colorSensor = sensor;
    }

    public int measureBrightness() {
        SampleProvider colorProvider = colorSensor.getRedMode();
        float[] sample = new float[colorProvider.sampleSize()];
        colorSensor.fetchSample(sample, 0);
        int brightness = (int) (sample[0] * 100);
        return brightness;
    }

    public void closeColorSensor() {
        colorSensor.close();
    }
}

class GyroSensorWrapper {
	private EV3GyroSensor gyroSensor;
	private SampleProvider angleProvider;
	private float[] angleSample;
	
	public GyroSensorWrapper(Port port) {
		gyroSensor = new EV3GyroSensor(port);
		angleProvider = gyroSensor.getAngleMode();
		angleSample = new float[angleProvider.sampleSize()];
		reset();
	}
	
	public void reset() {
		gyroSensor.reset();
	}
	
	public float getAngle() {
		angleProvider.fetchSample(angleSample, 0);
		return angleSample[0];
	}
	
	public void close() {
		gyroSensor.close();
	}	
}

class Odometrie {
    private List<Movement> movements = new ArrayList<>();
    private GyroSensorWrapper gyroSensor;
    private int initialTachoLeft;
    private int initialTachoRight;
    private float lastAngle;

    public Odometrie(Port gyroPort) {
        gyroSensor = new GyroSensorWrapper(gyroPort);
    }

    public void startTracking() {
        initialTachoLeft = MotorController.motorLeft.getTachoCount();
        initialTachoRight = MotorController.motorRight.getTachoCount();
        lastAngle = gyroSensor.getAngle();
    }

    public void addDistance(int duration) {
        int currentTachoLeft = MotorController.motorLeft.getTachoCount();
        int currentTachoRight = MotorController.motorRight.getTachoCount();
        int distance = (currentTachoLeft - initialTachoLeft + currentTachoRight - initialTachoRight) / 2;
        if (distance != 0) {
            movements.add(new Movement(distance, 0, duration));
            initialTachoLeft = currentTachoLeft;
            initialTachoRight = currentTachoRight;
        }
    }

    public void addRotation(int duration) {
        float currentAngle = gyroSensor.getAngle();
        float angleChange = currentAngle - lastAngle;
        if (angleChange != 0) {
            movements.add(new Movement(0, angleChange, duration));
            lastAngle = currentAngle;
        }
    }

    public void stopTracking() {
        gyroSensor.close();
    }

    public List<Movement> getMovements() {
        return movements;
    }

    public void clearMovements() {
        movements.clear();
    }

    class Movement {
        int distance;
        float angle;
        int duration;

        Movement(int distance, float angle, int duration) {
            this.distance = distance;
            this.angle = angle;
            this.duration = duration;
        }
    }

    public static void returnToStartWithCancelOption(Odometrie odometrie) {
        TextLCD lcd = LocalEV3.get().getTextLCD();
        lcd.clear();
        lcd.drawString("Returning to start", 0, 0);
        lcd.drawString("Press ESC to cancel", 0, 1);

        List<Movement> reversedMovements = new ArrayList<>();
        List<Movement> originalMovements = odometrie.getMovements();

        int distanceCount = 0;
        int angleCount = 0;
        for (int i = originalMovements.size() - 1; i >= 0; --i) {
            Movement mov = originalMovements.get(i);
            if (mov.distance != 0) ++distanceCount;
            if (mov.angle != 0) ++angleCount;
            reversedMovements.add(mov);
        }

        lcd.drawString("Distance: " + distanceCount, 0, 2);
        lcd.drawString("Angles: " + angleCount, 0, 3);

        MotorController mc = new MotorController();
        mc.pause(500);

        for (int i = 0; i < reversedMovements.size(); ++i) {
            mc.pause(500);
            Movement mov = reversedMovements.get(i);

            if (mov.angle != 0) {
                MotorController.motorLeft.setSpeed(MotorController.manualTurnSpeed);
                MotorController.motorRight.setSpeed(MotorController.manualTurnSpeed);
                if (mov.angle < 0) { // Das war eine Linksdrehung. Deshalb jetzt Rechtsdrehung
                    MotorController.motorLeft.backward();
                    MotorController.motorRight.forward();
                } else {
                    MotorController.motorLeft.forward();
                    MotorController.motorRight.backward();
                }
                MotorController.pause(mov.duration);
                MotorController.motorLeft.stop(true);
                MotorController.motorRight.stop(true);
            }

            if (mov.distance != 0) {
                boolean opposideMovement = mov.distance > 0;
                mc.dynamicSpeedAdjustment(!opposideMovement, mov.duration);
            }

            // Anzeige der Tacho-Werte nach der Bewegung
            lcd.clear();
            lcd.drawString("Tacho L: " + MotorController.motorLeft.getTachoCount(), 0, 0);
            lcd.drawString("Tacho R: " + MotorController.motorRight.getTachoCount(), 0, 1);

            if (Button.ESCAPE.isDown()) {
                lcd.clear();
                lcd.drawString("Aborted", 0, 0);
                break;
            }
        }

        lcd.clear();
        lcd.drawString("Returned to start", 0, 0);
    }

}

class MotorController {
    public static EV3LargeRegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.B);
    public static EV3LargeRegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.C);
    public static MeasureDistance measureDistance = new MeasureDistance(SensorPort.S4);
    public static Odometrie odometrie = new Odometrie(SensorPort.S2);
    public static int startSpeed = 250;
    public static int speedManual = 350; // Das Fahrzeug muss immer diese Geschwindigkeit beim Fahren haben: nach Vorne fahren & nach Hinten fahrne
    public static int manualSpeed = 500;
    public static int manualTurnSpeed = 70; // Das ist die Geschwindigkeit für die Drehung 
    public static int manualTurnDuration = 5000; // Das isg die Dauer für die Drehung
    public static final float SAFE_DISTANCE = 0.25f;
    private static final Object lock = new Object();
    static int roundCounter = 0;

    public static void initializeAndStartMotors() {
        motorLeft.resetTachoCount();
        motorRight.resetTachoCount();
        motorLeft.setSpeed(100);
        motorRight.setSpeed(100);
        motorLeft.forward();
        motorRight.forward();
        odometrie.startTracking();
    }

    public static void incrementMotorSpeeds(int targetSpeedLeft, int targetSpeedRight) {
        int currentSpeedLeft = motorLeft.getSpeed();
        int currentSpeedRight = motorRight.getSpeed();
        int increment = 10;

        MeasureBrightness measure = new MeasureBrightness(DisplayInfo.colorSensor);

        while (currentSpeedLeft != targetSpeedLeft || currentSpeedRight != targetSpeedRight) {
            if (PIDController.checkLineLost(measure)) return;
            synchronized (lock) {
                if (VehicleController.checkObstacleAndReact()) return;

                currentSpeedLeft = adjustSpeed(currentSpeedLeft, targetSpeedLeft, increment);
                if (PIDController.checkLineLost(measure)) return;
                if (VehicleController.checkObstacleAndReact()) return;

                currentSpeedRight = adjustSpeed(currentSpeedRight, targetSpeedRight, increment);
                if (PIDController.checkLineLost(measure)) return;
                if (VehicleController.checkObstacleAndReact()) return;

                setMotorSpeeds(currentSpeedLeft, currentSpeedRight);
            }
            pause(50);
        }
    }

    public static void setMotorSpeeds(int leftSpeed, int rightSpeed) {
        motorLeft.setSpeed(leftSpeed);
        motorRight.setSpeed(rightSpeed);
        motorLeft.forward();
        motorRight.forward();
    }

    public static void stopMotors() {
        motorLeft.stop(true);
        motorRight.stop(true);
        odometrie.stopTracking();
    }
    
    public static void recordDistance(int duration) {
    	odometrie.addDistance(duration);
    }
    
    public static void recordRotation(int duration) {
    	odometrie.addRotation(duration);
    }

    public static void closeConnection() {
        motorLeft.close();
        motorRight.close();
        odometrie.stopTracking();
    }

    private static int adjustSpeed(int currentSpeed, int targetSpeed, int increment) {
        if (currentSpeed < targetSpeed) {
            return Math.min(currentSpeed + increment, targetSpeed);
        } else {
            return Math.max(currentSpeed - increment, targetSpeed);
        }
    }
    
    

    public static void startManualControl() {
        TextLCD lcd = LocalEV3.get().getTextLCD();
        lcd.clear();
        lcd.drawString("Press any button", 0, 0);
        lcd.drawString("to start manual", 0, 1);
        lcd.drawString("control...", 0, 2);
        Button.waitForAnyPress();
        lcd.clear();
        //String up = "UP", left = "LEFT", right = "RIGHT", down = "DOWN", enter = "ENTER", esc = "ESC";
        //lcd.drawString(esc, 0, 0);
        //lcd.drawString(up, (lcd.getTextWidth() - up.length()) / 2, 0);
        //lcd.drawString(left, 0, 3);
        //lcd.drawString(enter, (lcd.getTextWidth() - enter.length()) / 2, lcd.getTextHeight() / 2);
        //lcd.drawString(right, lcd.getTextWidth() - right.length(), 3);
        //lcd.drawString(down, (lcd.getTextWidth() - down.length()) / 2, lcd.getTextHeight() - 1);
        odometrie.clearMovements();
        odometrie.startTracking();
        int timeDurationForMovement = 2000; // Total duration in milliseconds
        int interval = timeDurationForMovement / 50; // Interval duration for each 2%
        manualTurnSpeed = 70;
        manualTurnDuration = 2500;

        boolean controlling = true;
        while (controlling) {
            int button = Button.waitForAnyPress();
            if(button == Button.ID_ESCAPE) {
                lcd.clear();
                return;
            }

            switch (button) {
                case Button.ID_UP:
                	int randomizedTimeDurationForMovementForward = (int)(timeDurationForMovement * (0.2 + Math.random() * 1.8));
                	dynamicSpeedAdjustment(true, randomizedTimeDurationForMovementForward);
                    recordDistance(randomizedTimeDurationForMovementForward);
                    break;
                case Button.ID_DOWN:
                	int randomizedTimeDurationForMovementBackward = (int)(timeDurationForMovement * (0.2 + Math.random() * 1.8));
                    dynamicSpeedAdjustment(false, randomizedTimeDurationForMovementBackward);
                    recordDistance(randomizedTimeDurationForMovementBackward);
                    break;
                case Button.ID_LEFT:
                    int randomizedManualTurnDurationLeft = (int)(manualTurnDuration * (0.2 + Math.random() * 1.8));
                	motorLeft.setSpeed(manualTurnSpeed);
                    motorRight.setSpeed(manualTurnSpeed);
                    motorLeft.backward();
                    motorRight.forward();
                    pause(randomizedManualTurnDurationLeft);
                    motorLeft.stop(true);
                    motorRight.stop(true);
                    recordRotation(randomizedManualTurnDurationLeft);
                    break;
                case Button.ID_RIGHT:
                	int randomizedManualTurnDurationRight = (int)(manualTurnDuration * (0.2 + Math.random() * 1.8));
                    motorLeft.setSpeed(manualTurnSpeed);
                    motorRight.setSpeed(manualTurnSpeed);
                    motorLeft.forward();
                    motorRight.backward();
                    pause(randomizedManualTurnDurationRight);
                    motorLeft.stop(true);
                    motorRight.stop(true);
                    recordRotation(randomizedManualTurnDurationRight);
                    break;
                case Button.ID_ENTER:
                    controlling = false;
                    break;
            }

            // Anzeige der Tacho-Werte nach der Bewegung
            lcd.clear();
            lcd.drawString("Tacho L: " + MotorController.motorLeft.getTachoCount(), 0, 0);
            lcd.drawString("Tacho R: " + MotorController.motorRight.getTachoCount(), 0, 1);
        }
        // Rückkehr zur Startposition
        odometrie.stopTracking();
        odometrie.returnToStartWithCancelOption(odometrie);
    }
    
    private static SpeedPIDController speedPIDController = new SpeedPIDController(1, 0.1, 0.1);
    public static void dynamicSpeedAdjustment(boolean forward, int duration) {
        // Setze die konstante Geschwindigkeit für die Motoren
        int baseSpeed = MotorController.speedManual;
        MotorController.motorLeft.setSpeed(baseSpeed);
        MotorController.motorRight.setSpeed(baseSpeed);

        if (forward) {
            MotorController.motorLeft.forward();
            MotorController.motorRight.forward();
        } else {
            MotorController.motorLeft.backward();
            MotorController.motorRight.backward();
        }

        int interval = 100;
        int elapsed = 0;
        while (elapsed < duration) {
            MotorController.pause(interval);
            elapsed += interval;
            // Wende Fehlerkompensation an
            applyErrorCompensation(baseSpeed, forward);
        }
      
		
        // Stoppe die Motoren am Ende der Bewegung
        MotorController.motorLeft.stop(true);
        MotorController.motorRight.stop(true);
        
        ++roundCounter;
        if(roundCounter % 3 == 0)
        	resetOdometrieSettings();
    }

    public static void applyErrorCompensation(int baseSpeed, boolean forward) {
        int currentTachoLeft = MotorController.motorLeft.getTachoCount();
        int currentTachoRight = MotorController.motorRight.getTachoCount();

        // Berechne die Differenz zwischen den Tacho-Werten
        int error = currentTachoLeft - currentTachoRight;
        int adjustment = speedPIDController.getCorrection(error);
        // Begrenze die Anpassung
        int maxAdjustment = 50; // Maximaler Anpassungswert
        adjustment = Math.max(-maxAdjustment, Math.min(adjustment, maxAdjustment));
        
        

        int leftSpeed = baseSpeed, rightSpeed = baseSpeed;
        
        if(leftSpeed > rightSpeed) {
       	   leftSpeed -= (adjustment - 1);
           rightSpeed += (adjustment);
        } else if(leftSpeed < rightSpeed) {
        	leftSpeed += (adjustment + 1);
        	rightSpeed -= (adjustment);
        }

        MotorController.motorLeft.setSpeed(leftSpeed);
        MotorController.motorRight.setSpeed(rightSpeed);
    }
    
    private static void resetOdometrieSettings() {
    	MotorController.motorLeft.setSpeed(MotorController.manualSpeed);
    	MotorController.motorRight.setSpeed(MotorController.manualSpeed);
    	speedPIDController.reset();	
    }

    public static void pause(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }   
}

class SpeedPIDController{
	private double Kp, Ki, Kd, previousError, integral;
	
	public SpeedPIDController(double Kp, double Ki, double Kd) {
		this.Kp = Kp; this.Ki = Ki; this.Kd = Kd; this.previousError = 0; this.integral = 0;
	}
	
	public int getCorrection(int error) {
		integral += error;
		double derivative = error - previousError;
		previousError = error;
		int correction = (int) (Kp * error + Ki * integral + Kd * derivative);
		return correction;
	}
	
	public void reset() {
		previousError = 0;
		integral = 0;
	}
}


class PIDController {
    public static int targetThreshold;
    private static double Kp = 0.001;
    private static double Ki = 0.001;
    private static double Kd = 0.001;
    private static double previousError = 0;
    private static double integral = 0;

    public static void setThreshold(int target) {
        targetThreshold = target;
    }

    public static void pidRegler() {
        MeasureBrightness measure = new MeasureBrightness(DisplayInfo.colorSensor);
        int currentBrightness = measure.measureBrightness();
        int error = targetThreshold - currentBrightness;

        int output = calculatePIDoutput(error);

        if (currentBrightness < targetThreshold) {
            MotorController.stopMotors();
            VehicleController.searchForLine(false);
        } else {
            VehicleController.adjustMotors(output);
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

        int output = (int) (kpFaktor + kiFaktor + kdFaktor);
        return output;
    }

    public static boolean checkLineLost(MeasureBrightness measure) {
        if (measure.measureBrightness() < targetThreshold) {
            MotorController.stopMotors();
            VehicleController.searchForLine(false);
            return true;
        }
        return false;
    }
}

class VehicleController {
    private static final Object lock = new Object();

    public static void startVehicle(TextLCD lcd) {
        MotorController.initializeAndStartMotors();
        Thread pidThread = new Thread(() -> {
            while (Button.ENTER.isUp()) {
                synchronized (lock) {
                    PIDController.pidRegler();
                }
                MotorController.pause(50);
            }
        });
        pidThread.start();

        Thread distanceThread = new Thread(() -> {
            while (Button.ENTER.isUp()) {
                if (MotorController.measureDistance.getDistance() < MotorController.SAFE_DISTANCE) {
                    synchronized (lock) {
                        ObstacleAvoidance.avoidObstacle();
                    }
                }
                MotorController.pause(100);
            }
        });
        distanceThread.start();

        Button.ENTER.waitForPressAndRelease();
        MotorController.stopMotors();
    }

    public static void searchForLine(boolean findPathBack) {
        int leftTurnDuration = findPathBack ? 1500 : 700;
        int rightTurnDuration = findPathBack ? 700 : 1500;
        boolean lineFound = false;
        while (!lineFound) {
            turnLeft();

            lineFound = waitForLineDetection(leftTurnDuration / 100);
            if (lineFound) break;

            rightTurnDuration += 250;
            turnRight();

            lineFound = waitForLineDetection(rightTurnDuration / 100);
            if (lineFound) break;

            leftTurnDuration += 250;
        }
        MotorController.stopMotors();
        MotorController.pause(50);
    }

    private static void turnLeft() {
        MotorController.motorLeft.setSpeed(50);
        MotorController.motorRight.setSpeed(50);
        MotorController.motorLeft.forward();
        MotorController.motorRight.backward();
    }

    private static void turnRight() {
        MotorController.motorLeft.setSpeed(50);
        MotorController.motorRight.setSpeed(50);
        MotorController.motorLeft.backward();
        MotorController.motorRight.forward();
    }

    private static boolean waitForLineDetection(int intervals) {
        MeasureBrightness measure = new MeasureBrightness(DisplayInfo.colorSensor);
        for (int i = 0; i < intervals; ++i) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }

            if (measure.measureBrightness() >= PIDController.targetThreshold) {
                return true;
            }
        }
        return false;
    }

    public static void adjustMotors(int correction) {
        int newSpeedLeft = MotorController.startSpeed - correction;
        int newSpeedRight = MotorController.startSpeed + correction;
        MotorController.pause(250);
        MotorController.incrementMotorSpeeds(newSpeedLeft, newSpeedRight);
    }

    public static boolean checkObstacleAndReact() {
        if (MotorController.measureDistance.getDistance() < MotorController.SAFE_DISTANCE) {
            ObstacleAvoidance.avoidObstacle();
            return true;
        }
        return false;
    }
}

class ObstacleAvoidance {
    public static int remainingTurnDuration = 1900;
    private static int findPathBackDuration = 5000;
    private static final Object lock = new Object();

    public static void avoidObstacle() {
        synchronized (lock) {
            turnRight90degree();
            moveForwardForTime(2300);
            if (turnLeft90degree(1)) {
                VehicleController.searchForLine(true);
                return;
            }
            makeChoice(4100);
            if (turnLeft90degree(2)) {
                VehicleController.searchForLine(true);
                return;
            }
            findPathBack(4100);
        }
    }

    private static void makeChoice(int milliseconds) {
        int result = moveForwardForTimeAndSearchBackForLine(milliseconds);
        if (result == 1) {
            VehicleController.searchForLine(true);
            return;
        }
        if (result == 2) {
            avoidObstacle();
            return;
        }
    }

    private static void findPathBack(int milliseconds) {
        int result = moveForwardForTimeAndSearchBackForLine(milliseconds);
        if (result == 1) {
            VehicleController.searchForLine(true);
            return;
        }
        if (result == 2) {
            avoidObstacle();
            return;
        }
        turnLeft90degree(2);
        findPathBack(findPathBackDuration);
    }

    private static void turnRight90degree() {
        int turnSpeed = 100;
        MotorController.motorLeft.setSpeed(turnSpeed);
        MotorController.motorRight.setSpeed(turnSpeed);
        MotorController.motorLeft.forward();
        MotorController.motorRight.backward();
        MotorController.pause(remainingTurnDuration);
        MotorController.motorLeft.stop(true);
        MotorController.motorRight.stop(true);
        MotorController.pause(100);
        remainingTurnDuration = Math.max(100, remainingTurnDuration);
    }

    private static boolean turnLeft90degree(int round) {
        int turnSpeed = 100;
        MotorController.motorLeft.setSpeed(turnSpeed);
        MotorController.motorRight.setSpeed(turnSpeed);
        MotorController.motorRight.forward();
        MotorController.motorLeft.backward();

        int duration = round == 1 ? 1800 : 1600;
        boolean lineFound = pauseAndCheckLine(duration);
        MotorController.motorLeft.stop(true);
        MotorController.motorRight.stop(true);
        MotorController.pause(100);
        return lineFound;
    }

    private static boolean pauseAndCheckLine(int milliseconds) {
        int interval = 100;
        int elapsed = 0;
        MeasureBrightness measure = new MeasureBrightness(DisplayInfo.colorSensor);

        while (elapsed < milliseconds) {
            if (measure.measureBrightness() >= PIDController.targetThreshold) {
                return true; // Linie gefunden
            }
            try {
                Thread.sleep(interval);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false; // Im Falle einer Unterbrechung zurückgeben
            }
            elapsed += interval;
        }
        return false; // Linie nicht gefunden
    }

    private static void moveForwardForTime(int milliseconds) {
        MotorController.pause(100);
        MotorController.motorLeft.setSpeed(MotorController.startSpeed);
        MotorController.motorRight.setSpeed(MotorController.startSpeed);
        MotorController.motorLeft.forward();
        MotorController.motorRight.forward();
        MotorController.pause(milliseconds);
    }

    private static int moveForwardForTimeAndSearchBackForLine(int milliseconds) {
        int initialSpeed = 100;
        int increment = 10;
        int interval = 100;
        int elapsed = 0;

        MeasureBrightness measure = new MeasureBrightness(DisplayInfo.colorSensor);

        MotorController.motorLeft.setSpeed(initialSpeed);
        MotorController.motorRight.setSpeed(initialSpeed);
        MotorController.motorLeft.forward();
        MotorController.motorRight.forward();

        while (elapsed < milliseconds) {
            if (measure.measureBrightness() >= PIDController.targetThreshold) {
                MotorController.motorLeft.stop(true);
                MotorController.motorRight.stop(true);
                return 1; // Linie gefunden
            }
            if (MotorController.measureDistance.getDistance() < MotorController.SAFE_DISTANCE) {
                MotorController.motorLeft.stop(true);
                MotorController.motorRight.stop(true);
                return 2; // Hindernis gefunden
            }
            if (MotorController.motorLeft.getSpeed() < MotorController.startSpeed) {
                int newSpeed = Math.min(MotorController.motorLeft.getSpeed() + increment, MotorController.startSpeed);
                MotorController.motorLeft.setSpeed(newSpeed);
                MotorController.motorRight.setSpeed(newSpeed);
            }

            MotorController.pause(interval);
            elapsed += interval;
        }

        MotorController.motorLeft.stop(true);
        MotorController.motorRight.stop(true);
        return 0; // Nichts gefunden
    }
}

/*
class PositionReceiver{
	private String serverUrl;
	
	public PositionReceiver(String serverUrl) {
		this.serverUrl = serverUrl;
	}
	
	public Position getCurrentPosition() {
		
	}
}

class Position{
	double x;
	double y;
	
	public Position(double x, double y) {
		this.x = x;
		this.y = y;
	}
}

class PathReceiver{
	private String serverUrl;
	
	public PathReceiver(String serverUrl) {
		this.serverUrl = serverUrl;
	}
	
	public List<Position> getPath(){
		
	}
}

class PathFollower{
	private PositionReceiver positionReceiver;
	private PathReceiver pathReceiver;
	private List<Position> path;
	private int currentPathIndex;
	
	public PathFollower(PositionReceiver positionReceiver, PathReceiver pathReceiver) {
		this.positionReceiver = positionReceiver;
		this.pathReceiver = pathReceiver;
		//this.path = pathReceiver.getPath();
		this.currentPathIndex = 0;
	}
	
	public void followPath() {
		while(currentPathIndex < path.size()) {
			Position currentPosition = positionReceiver.getCurrentPosition();
			Position targetPosition = path.get(currentPathIndex);
			
			double distance = calculateDistance(currentPosition, targetPosition);
			if(distance < 1.0) {
				++currentPathIndex;
				continue;
			}
			
			double angle = calculateAngle(currentPosition, targetPosition);
			adjustMotors(angle, distance);
			
			MotorController.pause(100); 
			}
		MotorController.stopMotors();
		}
	
	private double calculateDistance(Position cutten, Position target) {
		// Berechne die Entfernung zwischen der aktuellen Position und der Zielposition
	}
	
	private doouble calculateAngle(Position curren, Position target) {
		
	}
	
	private void adjustMotors(double angle, double distance) {
		// Steuerung der Motoren basierend auf dem berechneten Winkel un der Entfernung
	}
}
*/
class Calibrate {
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
        PIDController.setThreshold(targetThreshold);
        Button.waitForAnyPress();
        lcd.clear();
    }
}

class Controller {
    private TextLCD lcd = LocalEV3.get().getTextLCD();

    public Controller() {
        displayMenu();
    }

    public void displayMenu() {
        boolean go = true;
        try {
            do {
                lcd.clear();
                lcd.drawString("UP: Auto Mode", 0, 0);
                lcd.drawString("DOWN: Manual Mode", 0, 1);
                lcd.drawString("ESC to exit", 0, 2);

                int button = Button.waitForAnyPress();
                lcd.clear();
                switch (button) {
                    case Button.ID_UP:
                        startAutoMode();
                        break;
                    case Button.ID_DOWN:
                        MotorController.startManualControl();
                        break;
                    case Button.ID_ESCAPE:
                        go = false;
                        break;
                    default:
                        lcd.drawString("Select 1 or 2", 0, 2);
                        break;
                }
            } while (go);
        } catch (RuntimeException e) {
            System.out.println("Program exited: " + e.getMessage());
        }
    }

    private void startAutoMode() {
        boolean go = true;
        try {
            do {
                lcd.clear();
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
                }
            } while (go);
        } catch (RuntimeException e) {
            System.out.println("Return to menu: " + e.getMessage());
        }
    }
}

public class DisplayInfo {
    public static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);

    public static void main(String[] args) {
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            TextLCD lcd = LocalEV3.get().getTextLCD();
            lcd.drawString("Cleanup operation...", 0, 0);
            MotorController.stopMotors();
            MotorController.closeConnection();
            MeasureBrightness cSensor = new MeasureBrightness(colorSensor);
            cSensor.closeColorSensor();
        }));
        new Controller();
    }
}
