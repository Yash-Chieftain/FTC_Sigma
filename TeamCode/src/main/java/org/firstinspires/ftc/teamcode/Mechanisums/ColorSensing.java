package org.firstinspires.ftc.teamcode.Mechanisums;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utils.Artifact;

@Configurable
public class ColorSensing {
   public static double intakeColorSensorThreshold = 2;
   public static double shootingColorSensorThreshold = 4;
   public static double greenPWM = 0.5;
   public static double purplePWM = 0.7;
   public static double redPWM = 0.3;
   public static long blinkDurationMs = 1000;     // total blink time
   public static long blinkIntervalMs = 150;      // blink speed

   RevColorSensorV3 intakeColorSensor, shootingColorSensor;
   boolean intakeDetected, shootingDetected;
   NormalizedRGBA intakeRGBA, shootingRGBA;
   private long blinkStartTime = 0;
   private boolean isBlinking = false;
   private long lastBlinkToggle = 0;
   private boolean ledState = false;

   Servo led;

   public ColorSensing(HardwareMap hardwareMap){
      intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorSensor");
      shootingColorSensor = hardwareMap.get(RevColorSensorV3.class, "shootingColorSensor");
      led = hardwareMap.get(Servo.class, "led");

   }

   public void startBlink() {
      isBlinking = true;
      blinkStartTime = System.currentTimeMillis();
      lastBlinkToggle = blinkStartTime;
      ledState = false;
   }

   public void blink() {
      if (!isBlinking) return;

      long now = System.currentTimeMillis();

      if (now - blinkStartTime >= blinkDurationMs) {
         isBlinking = false;
         led.setPosition(0);
         return;
      }

      if (now - lastBlinkToggle >= blinkIntervalMs) {
         ledState = !ledState;
         led.setPosition(ledState ? redPWM : 0);
         lastBlinkToggle = now;
      }
   }

   @SuppressLint("DefaultLocale")
   public String update() {
      intakeRGBA = null;
      shootingRGBA = null;

      intakeDetected = intakeColorSensor.getDistance(DistanceUnit.CM) < intakeColorSensorThreshold;
      shootingDetected = shootingColorSensor.getDistance(DistanceUnit.CM) < shootingColorSensorThreshold;

      if (intakeDetected) {
         intakeRGBA = intakeColorSensor.getNormalizedColors();
         led.setPosition(intakeRGBA.green > intakeRGBA.blue ? greenPWM : purplePWM);
      } else if (shootingDetected) {
         shootingRGBA = shootingColorSensor.getNormalizedColors();
         led.setPosition(shootingRGBA.green > shootingRGBA.blue ? greenPWM : purplePWM);
      } else {
         if (!isBlinking) {
            led.setPosition(0);
         }
         return "No Color";
      }

      return String.format(
         "Intake: dist=%.2f r=%.2f g=%.2f b=%.2f | Shooting: dist=%.2f r=%.2f g=%.2f b=%.2f",
         intakeColorSensor.getDistance(DistanceUnit.CM),
         (intakeRGBA != null ? intakeRGBA.red : 0),
         (intakeRGBA != null ? intakeRGBA.green : 0),
         (intakeRGBA != null ? intakeRGBA.blue : 0),
         shootingColorSensor.getDistance(DistanceUnit.CM),
         (shootingRGBA != null ? shootingRGBA.red : 0),
         (shootingRGBA != null ? shootingRGBA.green : 0),
         (shootingRGBA != null ? shootingRGBA.blue : 0)
      );
   }


   public Artifact getIntakeColorDetected() {
      if (intakeDetected) {
         return (intakeRGBA.green > intakeRGBA.blue) ? Artifact.GREEN : Artifact.PURPLE;
      }
      return Artifact.EMPTY;
   }

   public Artifact getShootingColorDetected() {
      if (shootingDetected) {
         return (shootingRGBA.green > shootingRGBA.blue) ? Artifact.GREEN : Artifact.PURPLE;
      }
      return Artifact.EMPTY;
   }
}
