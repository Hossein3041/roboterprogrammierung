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

public class TestAufgabe2 {
    private static EV3LargeRegulatedMotor motorLinks = new EV3LargeRegulatedMotor(MotorPort.B);
    private static EV3LargeRegulatedMotor motorRechts = new EV3LargeRegulatedMotor(MotorPort.C);

    private static final int START_SPEED = 100;
    private static int LINIENSCHWELLE;
    private static int BODENSCHWELLE;
    private static int SOLLWERT;
    

    private static double KpTable = 2.7;
    private static double KiTable = 0.4;
    private static double KdTable = 3;

    private static double KpFloor = 15;
    private static double KiFloor = 0;
    private static double KdFloor = 0;

    private static double previousError = 0;
    private static double integral = 0;

    private static EV3ColorSensor colorSensor;

    public static void main(String[] args) {
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            System.out.println("Aufräumaktion...");
            stopMotors(motorLinks, motorRechts);
            closeConnection(motorLinks, motorRechts);
        }));
        tastenSteuerung();
    }

    private static void tastenSteuerung() {
        TextLCD lcd = LocalEV3.get().getTextLCD();
        lcd.drawString("Drücken Sie eine Taste", 0, 0);
        Button.waitForAnyPress();
        lcd.clear();

        lcd.drawString("Kalibrierung gestartet", 0, 0);
        Button.waitForAnyPress();
        lcd.clear();

        kalibrieren(lcd);

        lcd.drawString("Kalibrierung abgeschlossen", 0, 0);
        Button.waitForAnyPress();
        lcd.clear();

        lcd.drawString("ENTER zum Starten", 0, 0);
        Button.waitForAnyPress();
        startFahrzeug(lcd);
    }

    private static void kalibrieren(TextLCD lcd) {
        initialisiereSensor();

        lcd.drawString("Linienmessung:", 0, 0);
        lcd.drawString("Taste Drücken", 0, 1);
        Button.waitForAnyPress();
        lcd.clear();
        LINIENSCHWELLE = farbmessungLinie();
        lcd.drawString("LinienH: " + LINIENSCHWELLE, 0, 0);
        Button.waitForAnyPress();
        lcd.clear();

        lcd.drawString("Bodenmessung:", 0, 0);
        lcd.drawString("Drücken Sie eine Taste", 0, 1);
        Button.waitForAnyPress();
        lcd.clear();
        BODENSCHWELLE = farbmessungBoden();
        lcd.drawString("BodenH " + BODENSCHWELLE, 0, 0);
        Button.waitForAnyPress();
        lcd.clear();

        SOLLWERT = (LINIENSCHWELLE + BODENSCHWELLE) / 2;
        lcd.drawString("Sollwert: " + SOLLWERT, 0, 0);
        Button.waitForAnyPress();
        lcd.clear();
    }

    private static void initialisiereSensor() {
        if (colorSensor == null)
            colorSensor = new EV3ColorSensor(SensorPort.S3);
    }

    private static int farbmessungLinie() {
        SampleProvider colorProvider = colorSensor.getRedMode();
        float[] sample = new float[colorProvider.sampleSize()];
        colorProvider.fetchSample(sample, 0);

        int helligkeit = (int) (sample[0] * 100);
        return helligkeit;
    }

    private static int farbmessungBoden() {
        SampleProvider colorProvider = colorSensor.getRedMode();
        float[] sample = new float[colorProvider.sampleSize()];
        colorProvider.fetchSample(sample, 0);

        int helligkeit = (int) (sample[0] * 100);
        return helligkeit;
    }

    private static void startFahrzeug(TextLCD lcd) {
        motorLinks.resetTachoCount();
        motorRechts.resetTachoCount();
        motorLinks.setSpeed(START_SPEED);
        motorRechts.setSpeed(START_SPEED);

        motorLinks.forward();
        motorRechts.forward();

        Thread pidThread = new Thread(() -> {
            while (Button.ENTER.isUp()) {
                if (isOnTable()) {
                    pidReglerTable();
                } else {
                    pidReglerFloor();
                }
                try {
                    Thread.sleep(30);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        pidThread.start();

        Button.ENTER.waitForPressAndRelease();
        stopMotors(motorLinks, motorRechts);
    }

    private static boolean isOnTable() {
        return BODENSCHWELLE > 20; // Adjust this condition based on your criteria for determining the table or floor
    }

    private static void pidReglerTable() {
        performPIDControl(KpTable, KiTable, KdTable);
    }

    private static void pidReglerFloor() {
        // Adjust PID gains based on current conditions
       /* if (aktuelleHelligkeit() < BODENSCHWELLE) {
            KpFloor = 20;
            KiFloor = 0.9;
            KdFloor = 0.3;
        } else {
            KpFloor = 80;
            KiFloor = 0.001;
            KdFloor = 0.04;
        }*/
        performPIDControl(KpFloor, KiFloor, KdFloor);
    }


    private static void performPIDControl(double Kp, double Ki, double Kd) {
        int aktHelligkeit = aktuelleHelligkeit();
        int fehler = aktHelligkeit - SOLLWERT;

        integral += fehler;

        double derivative = (fehler - previousError);

        int maxIntegral = 100;
        integral = Math.max(Math.min(integral, maxIntegral), -maxIntegral);

        int output = (int) (Kp * fehler + Ki * integral + Kd * derivative);
        previousError = fehler;

        motorenAnpassen(output);
        return;
    }

    private static int aktuelleHelligkeit() {
        SampleProvider colorProvider = colorSensor.getRedMode();
        float[] sample = new float[colorProvider.sampleSize()];
        colorProvider.fetchSample(sample, 0);
        int helligkeit = (int) (sample[0] * 100);

        return helligkeit;
    }

    private static void motorenAnpassen(int korrektur) {
        int neueGeschwindigkeitLinks = START_SPEED - korrektur;
        int neueGeschwindigkeitRechts = START_SPEED + korrektur;

        neueGeschwindigkeitLinks = Math.max(-900, Math.min(900, neueGeschwindigkeitLinks));
        neueGeschwindigkeitRechts = Math.max(-900, Math.min(900, neueGeschwindigkeitRechts));

        motorLinks.setSpeed(neueGeschwindigkeitLinks);
        motorRechts.setSpeed(neueGeschwindigkeitRechts);

        motorLinks.forward();
        motorRechts.forward();
    }

    private static void beenden(TextLCD lcd) {
        lcd.clear();
        lcd.drawString("Programm beendet", 0, 0);
        stopMotors(motorLinks, motorRechts);
        closeConnection(motorLinks, motorRechts);
    }

    private static void stopMotors(EV3LargeRegulatedMotor... motors) {
        for (EV3LargeRegulatedMotor motor : motors) {
            motor.stop();
        }
    }

    private static void closeConnection(EV3LargeRegulatedMotor... motors) {
        for (EV3LargeRegulatedMotor motor : motors) {
            motor.close();
        }
    }
}
