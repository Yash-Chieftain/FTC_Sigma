package org.firstinspires.ftc.teamcode.Mechanisums;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.Artifact;
import org.firstinspires.ftc.teamcode.Utils.ShootingSequence;
import org.firstinspires.ftc.teamcode.Utils.Wait;

@Configurable
public class Mechanisms {
   public static long movementSleep = 200;
   public static long actionWait = 300;
   Intake intake;
   Shooter shooter;
   SpinIndexer spinIndexer;
   ColorSensing colorSensing;
   Vision vision;
   Turret turret;
   Parking parking;
   ElapsedTime shootTimer = new ElapsedTime();
   ElapsedTime intakeTimer = new ElapsedTime();
   int[] validateIntake = {1, 2, 0};
   boolean doIncrement = true;
   boolean isMovementDone = false;
   private int shootState = -1;
   private int intakeState = 0;
   private ShootingSequence[] shootingSequences;

   public Mechanisms(HardwareMap hardwareMap) {
      intake = new Intake(hardwareMap);
      colorSensing = new ColorSensing(hardwareMap);
      shooter = new Shooter(hardwareMap);
      spinIndexer = new SpinIndexer(hardwareMap);
      vision = new Vision(hardwareMap);
      turret = new Turret(hardwareMap);
      parking = new Parking(hardwareMap);
   }

   //Intake
   public void stopIntake() {
      intake.stopIntake();
   }

   public void slowIntake() {
      intake.slowIntake();
   }

   public void reverseIntake() {
      intake.reverse();
   }


   public void startIntake() {
      this.update();
      intake.startIntake();
      if (!spinIndexer.getIsCurrentIntake()) {
         spinIndexer.setPosition(1);
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

         } else {
            intake.startIntake();
         }
      }
   }


   //Shooter

   public void startShooter() {
      shooter.startShooter();
      intake.startIntake();
   }

   public void startLongshooter() {
      shooter.startLongshooter();
      intake.startIntake();
   }

   public void setShooterhood(double position) {
      shooter.setHoodPosition(position);
   }

   public void startShortShooter() {
      shooter.startShortShooter();
   }


   public void rampUpShooter() {
      shooter.startShooter();
   }

   public double getTargetVelocity() {
      return shooter.getTargetVelocity();
   }

   public void stopShooter() {
      shooter.stopShooter();
   }

   public double getHoodPosition() {
      return shooter.getHoodPosition();
   }

   public void readyToShoot(Artifact[] pattern) {
      if (spinIndexer.getArtifact(Artifact.ANY) == -1) {
         shootState = -1;
         shooter.stopShooter();
         return;
      }
      if (shootState == -1) {
         shootState = 0;
         shootingSequences = spinIndexer.getBestShootingSequence(pattern);
         shootTimer.reset();
      }

      if (shootingSequences.length - 1 < shootState) {
         shootState = -1;
         shooter.stopShooter();
         return;
      }

      spinIndexer.setPosition(shootingSequences[shootState].section, shootingSequences[shootState].shootingIndex);
   }

   public boolean shoot(Artifact[] pattern) {
      //this function will return true when there is no artifact left
      if (spinIndexer.getArtifact(Artifact.ANY) == -1) {
         shootState = -1;
         shooter.stopShooter();
         return true;
      }
      if (shootState == -1) {
         shootState = 0;
         shootingSequences = spinIndexer.getBestShootingSequence(pattern);
         shootTimer.reset();
      }

      if (shootingSequences.length - 1 < shootState) {
         shootState = -1;
         shooter.stopShooter();
         return true;
      }

      spinIndexer.setPosition(shootingSequences[shootState].section, shootingSequences[shootState].shootingIndex);
      shooter.startShooter();
      intake.startIntake();
      if (colorSensing.shootingDetected) {
         shootTimer.reset();
      }


      if ((shooter.isVelocityReached() && colorSensing.shootingDetected) || shootTimer.milliseconds() > 1000) {
         Wait.mySleep(actionWait);
         while (!shooter.shoot()) ;
         shootTimer.reset();
         spinIndexer.setSectionArtifact(shootingSequences[shootState].section, Artifact.EMPTY);
         shootState++;
      }
      if (shootState > 2) {
         shooter.stopShooter();
         shootState = -1;
         return true;
      }
      this.update();

      return shootState == -1 || shootState > 2;
   }

   public void shoot() {
      if (shooter.shoot()) {
         spinIndexer.setSectionArtifact(spinIndexer.getCurrentPosition(), Artifact.EMPTY);
      }
   }

   //SpinIndexer
   public void validateArtifacts() {
//      if( colorSensing.getIntakeColorDetected() &&  intakeTimer.milliseconds()  200){
//
//      }
      if (intakeState > 2) {
         intakeTimer.reset();
         doIncrement = false;
         intakeState--;
      }
      if (intakeState < 0) {
         intakeTimer.reset();
         doIncrement = true;
         intakeState++;
      }
      spinIndexer.setPosition(validateIntake[intakeState]);
      if (intakeTimer.milliseconds() > 2000) {
         spinIndexer.setSectionArtifact(
            validateIntake[intakeState],
            Artifact.EMPTY
         );
         if (doIncrement) {
            intakeState++;
         } else {
            intakeState--;
         }
         if (intakeState > 2) {
            intakeTimer.reset();
            doIncrement = false;
            intakeState--;
         }
         intakeTimer.reset();
      }
      Artifact intakeColorDetected = colorSensing.getIntakeColorDetected();
      if (intakeColorDetected != Artifact.EMPTY && intakeColorDetected != Artifact.ANY) {
         intake.slowIntake();
         spinIndexer.setSectionArtifact(
            validateIntake[intakeState],
            intakeColorDetected
         );
         intakeState++;
         if (intakeState > 2) {
            intakeState = 0;
         }
         spinIndexer.setPosition(validateIntake[intakeState]);
         intakeTimer.reset();
         Wait.mySleep(movementSleep);
      }
   }


   public int getNoOfArtifacts() {
      return spinIndexer.getNoOfArtifacts();
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

   public void setSpinIndexerShootingPosition(int position) {
      spinIndexer.setPosition(position, 0);
   }

   public void setSpinIndexerIntakePosition(int position) {
      spinIndexer.setPosition(position);
   }

   public void setSpinIndexerState(Artifact[] state) {
      for (int i = 0; i < 3; i++) {
         spinIndexer.setSectionArtifact(i, state[i]);
      }
   }

   public void resetturret() {
      turret.reset();
   }

   public String getShootingSequenceString(Artifact[] pattern) {
      return spinIndexer.getShootingSequenceString(pattern);
   }

   //Vision
   public double getTurnValue() {
      return vision.alignTurnValue(0);
   }

   public double getDistance() {
      return vision.getDistance();
   }

   public double getTy() {
      return vision.getTy();
   }

   public int getId() {
      return vision.getID();
   }

   public void pipelineSwitch(int index) {
      vision.pipelineSwitch(index);
   }

   //Turret

   public void setTurretTicks(int ticks) {
      turret.setTurretTicks(ticks);
   }
   public boolean isTurretBusy(){
      return turret.isBusy();
   }
   public boolean isTurretAligned(){
      return (Turret.turretOffset - vision.getTx()) < 1 && vision.update();
   }

   //Parking
   public void park() {
      parking.park();
   }

   public void unPark() {
      parking.reset();
   }


   public void update() {
      colorSensing.update();

      shooter.setShooter(vision.getDistance());
      vision.update();

      if (vision.update() && Math.abs(Turret.turretOffset - vision.getTx()) > 1) {
         turret.alignLimeLight(vision.getTx());
      } else {
         turret.setPower(0);
      }
      Turret.updateTurretPosition((int) turret.getTicks());
      if (vision.update()) {
         shooter.setShooter(vision.getDistance());
      }
   }
}