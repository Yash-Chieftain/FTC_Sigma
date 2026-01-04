//package org.firstinspires.ftc.teamcode.Mechanisums;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.Utils.Artifact;
//import org.firstinspires.ftc.teamcode.Utils.Wait;
//
//public class Mechanisums {
//   public static long movementSleep = 200;
//   Intake intake;
//   Shooter shooter;
//   SpinIndexer spinIndexer;
//   ColorSensing colorSensing;
//   private int shootState = 0;
//   public static long actionWait = 200;
//
//   public Mechanisums(HardwareMap hardwareMap) {
//      intake = new Intake(hardwareMap);
//      colorSensing = new ColorSensing(hardwareMap);
//      shooter = new Shooter(hardwareMap);
//      spinIndexer = new SpinIndexer(hardwareMap);
//
//   }
//
//   public boolean shoot(Artifact[] sequence) {
//      //this function will return true when there is no artifact left
//      if (spinIndexer.getArtifact(Artifact.ANY) == -1) {
//         shooter.stopShooter();
//         return true;
//      }
//      if (shootState == -1) {
//         shootState = 0;
//      }
//      spinIndexer.setPosition(shootState, 0);
//      shooter.startShooter();
//      intake.slowIntake();
//
//      if (shooter.isVelocityReached() && colorSensing.shootingDetected) {
//         Wait.mySleep(actionWait);
//         shooter.shoot();
//         while (colorSensing.shootingDetected){
//            shooter.shoot();
//            this.update();
//         }
//         spinIndexer.setSectionArtifact(shootState, Artifact.EMPTY);
//         shootState++;
//         Wait.mySleep(actionWait);
//      }
//
//      if (shootState == 3) {
//         shooter.stopShooter();
//         shootState = -1;
//      }
//
//      return shootState == -1;
//   }
//
//   public void startIntake() {
//      if (!spinIndexer.getIsCurrentIntake()) {
//         spinIndexer.setPosition(Artifact.EMPTY, true);
//      }
//
//      Artifact intakeColorDetected = colorSensing.getIntakeColorDetected();
//      if (intakeColorDetected != Artifact.EMPTY) {
//         intake.slowIntake();
//         spinIndexer.setSectionArtifact(
//            spinIndexer.getCurrentPosition(),
//            intakeColorDetected
//         );
//         spinIndexer.setPosition(Artifact.EMPTY, true);
//         Wait.mySleep(movementSleep);
//      }
//
//      if (!spinIndexer.setPosition(Artifact.EMPTY, true)) {
//         intake.stopIntake();
//         intake.reverse();
//      } else {
//         intake.reset();
//         intake.startIntake();
//      }
//
//   }
//
//   public void slowIntake() {
//      intake.slowIntake();
//   }
//
//   public String getState() {
//      return spinIndexer.getSpinIndexerState();
//   }
//
//   public String getColorSensorValues() {
//      return colorSensing.update();
//   }
//
//   public double getShooterVelocity(){
//      return shooter.getVelocity();
//   }
//
//   public void setSpinIndexerState(Artifact[] state){
//      for(int i = 0; i < 3; i++){
//         spinIndexer.setSectionArtifact(i, state[i]);
//      }
//   }
//
//   public void update(){
//      colorSensing.update();
//   }
//}
