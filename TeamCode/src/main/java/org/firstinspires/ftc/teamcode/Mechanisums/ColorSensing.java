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
   public static double intakeColorSensorThreshold = 4;
   public static double shootingColorSensorThreshold = 6;
   public static double greenPWM = 0.5;
   public static double purplePWM = 0.7;
   public static double redPWM = 0.3;
   public static long blinkDurationMs = 1000;     // total blink time
   public static long blinkIntervalMs = 150;      // blink speed

   RevColorSensorV3 intakeColorSensor1, intakeColorSensor2, shootingColorSensor1,shootingColorSensor2;
   boolean intakeDetected1, intakeDetected2, shootingDetected;
   NormalizedRGBA intakeRGBA1, intakeRGBA2, shootingRGBA1,shootingRGBA2 ;
   Servo led;
   private long blinkStartTime = 0;
   private boolean isBlinking = false;
   private long lastBlinkToggle = 0;
   private boolean ledState = false;

   public ColorSensing(HardwareMap hardwareMap) {
      intakeColorSensor1 = hardwareMap.get(RevColorSensorV3.class, "intakeColorSensor1");
      intakeColorSensor2 = hardwareMap.get(RevColorSensorV3.class, "intakeColorSensor2");
      shootingColorSensor1 = hardwareMap.get(RevColorSensorV3.class, "shootingColorSensor1");
      shootingColorSensor2 = hardwareMap.get(RevColorSensorV3.class, "shootingColorSensor2");
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
      intakeRGBA1 = null;
      intakeRGBA2 = null;
      shootingRGBA1 = null;
      shootingRGBA2 = null;

      intakeDetected1 = intakeColorSensor1.getDistance(DistanceUnit.CM) < intakeColorSensorThreshold;
      intakeDetected2 = intakeColorSensor2.getDistance(DistanceUnit.CM) < intakeColorSensorThreshold;
      shootingDetected = shootingColorSensor1.getDistance(DistanceUnit.CM) < shootingColorSensorThreshold || shootingColorSensor2.getDistance(DistanceUnit.CM) < shootingColorSensorThreshold;

      if (intakeDetected1 && intakeDetected2) {
         intakeRGBA1 = intakeColorSensor1.getNormalizedColors();
         intakeRGBA2 = intakeColorSensor2.getNormalizedColors();

         led.setPosition(
            intakeRGBA1.green > intakeRGBA1.blue ? greenPWM : purplePWM
         );

      } else if (shootingDetected) {
         shootingRGBA1 = shootingColorSensor1.getNormalizedColors();
         shootingRGBA2 = shootingColorSensor2.getNormalizedColors();

         led.setPosition(
            shootingRGBA1.green > shootingRGBA1.blue ? greenPWM : purplePWM
         );

      } else {
         if (!isBlinking) {
            led.setPosition(0);
         }
         return "No Color";
      }

      return String.format(
         "Intake1: dist=%.2f r=%.2f g=%.2f b=%.2f | " +
            "Intake2: dist=%.2f r=%.2f g=%.2f b=%.2f | " +
            "Shooting1: dist=%.2f r=%.2f g=%.2f b=%.2f"+
            "Shooting2: dist=%.2f r=%.2f g=%.2f b=%.2f",
         intakeColorSensor1.getDistance(DistanceUnit.CM),
         (intakeRGBA1 != null ? intakeRGBA1.red : 0),
         (intakeRGBA1 != null ? intakeRGBA1.green : 0),
         (intakeRGBA1 != null ? intakeRGBA1.blue : 0),
         intakeColorSensor2.getDistance(DistanceUnit.CM),
         (intakeRGBA2 != null ? intakeRGBA2.red : 0),
         (intakeRGBA2 != null ? intakeRGBA2.green : 0),
         (intakeRGBA2 != null ? intakeRGBA2.blue : 0),
         shootingColorSensor1.getDistance(DistanceUnit.CM),
         (shootingRGBA1 != null ? shootingRGBA1.red : 0),
         (shootingRGBA1 != null ? shootingRGBA1.green : 0),
         (shootingRGBA1 != null ? shootingRGBA1.blue : 0),
         shootingColorSensor2.getDistance(DistanceUnit.CM),
         (shootingRGBA2 != null ? shootingRGBA2.red : 0),
         (shootingRGBA2 != null ? shootingRGBA2.green : 0),
         (shootingRGBA2 != null ? shootingRGBA2.blue : 0)
      );
   }


   public Artifact getIntakeColorDetected() {
      this.update();
      Artifact intake1, intake2;
      if (intakeDetected1 && intakeDetected2) {
         intake1 = (intakeRGBA1.green > intakeRGBA1.blue) ? Artifact.GREEN : Artifact.PURPLE;
         intake2 = (intakeRGBA2.green > intakeRGBA2.blue) ? Artifact.GREEN : Artifact.PURPLE;
         if (intake1 == intake2) {
            return intake1;
         } else {
            return Artifact.ANY;
         }
      }
      return Artifact.EMPTY;
   }

   public Artifact getShootingColorDetected() {
      this.update();
      Artifact shooting1, shooting2;
      if (shootingDetected) {
         shooting1 = (shootingRGBA1.green > shootingRGBA1.blue) ? Artifact.GREEN : Artifact.PURPLE;
         shooting2 = (shootingRGBA2.green > shootingRGBA2.blue) ? Artifact.GREEN : Artifact.PURPLE;

         if (shooting1 == shooting2) {
            return shooting1;
         } else {
            return Artifact.ANY;
         }
      }
      return Artifact.EMPTY;
   }
}
