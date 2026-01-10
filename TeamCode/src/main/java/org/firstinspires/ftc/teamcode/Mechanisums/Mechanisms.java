package org.firstinspires.ftc.teamcode.Mechanisums;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.Artifact;
import org.firstinspires.ftc.teamcode.Utils.Wait;

public class Mechanisms {
   public static long movementSleep = 200;
   public static long actionWait = 200;
   Intake intake;
   Shooter shooter;
   SpinIndexer spinIndexer;
   ColorSensing colorSensing;
   Vision vision;
   Turret turret;
   Parking parking;
   ElapsedTime shootTimer = new ElapsedTime();
   ElapsedTime intakeTimer = new ElapsedTime();
   private int shootState = 0;
   private int intakeState = 0;

   public Mechanisms(HardwareMap hardwareMap) {
      intake = new Intake(hardwareMap);
      colorSensing = new ColorSensing(hardwareMap);
      shooter = new Shooter(hardwareMap);
      spinIndexer = new SpinIndexer(hardwareMap);
      vision = new Vision(hardwareMap);
      turret = new Turret(hardwareMap);
      parking = new Parking(hardwareMap);
   }

   public double getTurnValue() {
      return vision.alignTurnValue(0);
   }

   public boolean shoot(Artifact[] sequence) {
      //this function will return true when there is no artifact left
      if (spinIndexer.getArtifact(Artifact.ANY) == -1) {
         shootState = -1;
         shooter.stopShooter();
         return true;
      }
      if (shootState == -1) {
         shootState = 0;
         shootTimer.reset();
      }
      if (spinIndexer.getArtifact(shootState) == Artifact.EMPTY) {
         shootState++;
         shootTimer.reset();
         return false;
      }
      spinIndexer.setPosition(shootState, 0);
      shooter.startShooter();
      if (colorSensing.shootingDetected) {
         shootTimer.reset();
      }

      if (shooter.isVelocityReached() && colorSensing.shootingDetected) {
         Wait.mySleep(actionWait);
         shooter.shoot();
         while (colorSensing.shootingDetected && shootTimer.milliseconds() < 1000) {
            shooter.shoot();
            this.update();
         }
         shootTimer.reset();
         spinIndexer.setSectionArtifact(shootState, Artifact.EMPTY);
         shootState++;
         Wait.mySleep(actionWait);
      }


      if (shootState > 2) {
         shooter.stopShooter();
         shootState = -1;
         return true;
      }
      this.update();

      return shootState == -1 || shootState > 2;
   }

   public void shoot(){
      shooter.shoot();
   }

   public void validateArtifacts() {
      if (intakeState > 2) {
         intakeState = -1;
      }
      if (intakeState == -1) {
         intakeTimer.reset();
         intakeState = 0;
      }
      spinIndexer.setPosition(intakeState);
      if (intakeTimer.milliseconds() > 2000) {
         spinIndexer.setSectionArtifact(
            intakeState,
            Artifact.EMPTY
         );
         intakeState++;
         if (intakeState > 2) {
            intakeTimer.reset();
            intakeState = 0;
         }
         intakeTimer.reset();
      }
      Artifact intakeColorDetected = colorSensing.getIntakeColorDetected();
      if (intakeColorDetected != Artifact.EMPTY && intakeColorDetected != Artifact.ANY) {
         intake.slowIntake();
         spinIndexer.setSectionArtifact(
            intakeState,
            intakeColorDetected
         );
         intakeState++;
         if (intakeState > 2) {
            intakeState = 0;
         }
         spinIndexer.setPosition(intakeState);
         intakeTimer.reset();
         Wait.mySleep(movementSleep);
      }


   }

   public void park(){
      parking.park();
   }
   public void unPark(){
      parking.reset();
   }

   public void startIntake() {
      intake.startIntake();
      if (!spinIndexer.getIsCurrentIntake()) {
         spinIndexer.setPosition(Artifact.EMPTY, true);
      }

      Artifact intakeColorDetected = colorSensing.getIntakeColorDetected();
      if (intakeColorDetected != Artifact.EMPTY && intakeColorDetected != Artifact.ANY) {
         spinIndexer.setSectionArtifact(
            spinIndexer.getCurrentPosition(),
            intakeColorDetected
         );
         spinIndexer.setPosition(Artifact.EMPTY, true);
         Wait.mySleep(movementSleep);

         if (!spinIndexer.setPosition(Artifact.EMPTY, true)) {
            intake.stopIntake();
            intake.reverse();
         } else {
            intake.reset();
            intake.startIntake();
         }
      }
   }

   public void stopIntake() {
      intake.stopIntake();
   }

   public void slowIntake() {
      intake.slowIntake();
   }
   public void reverseIntake(){
      intake.reverse();
   }

   public String getState() {
      return spinIndexer.getSpinIndexerState();
   }

   public String getColorSensorValues() {
      return colorSensing.update();
   }

   public double getShooterVelocity() {
      return shooter.getVelocity();
   }

   public void setSpinIndexerState(Artifact[] state) {
      for (int i = 0; i < 3; i++) {
         spinIndexer.setSectionArtifact(i, state[i]);
      }
   }

   public void startShooter() {
      shooter.startShooter();
   }

   public void stopShooter() {
      shooter.stopShooter();
   }

   public void update() {
      colorSensing.update();
      if (vision.update() && Math.abs(vision.getTx())> 2) {
         turret.alignLimeLight(vision.getTx());
      }
      else {
         turret.alignLimeLight(0);
      }
   }
}
