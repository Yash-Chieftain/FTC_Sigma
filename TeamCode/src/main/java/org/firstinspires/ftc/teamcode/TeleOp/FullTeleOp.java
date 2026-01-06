package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisums.ColorSensing;
import org.firstinspires.ftc.teamcode.Mechanisums.Intake;
import org.firstinspires.ftc.teamcode.Mechanisums.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisums.SpinIndexer;
import org.firstinspires.ftc.teamcode.Utils.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class FullTeleOp extends LinearOpMode {
   Follower follower;
   Intake intake;
   Shooter shooter;
   ColorSensing colorSensing;
   SpinIndexer spinIndexer;
   double maxPower = 1;
   int spinIndexerIndex = 0;
   ElapsedTime spinIndexerMovementTimer = new ElapsedTime();
   DcMotor turret;

   @Override
   public void runOpMode() throws InterruptedException {
      turret = hardwareMap.get(DcMotor.class, "turret");
      turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      turret.setPower(0);
      follower = Constants.createFollower(hardwareMap);
      intake = new Intake(hardwareMap);
      shooter = new Shooter(hardwareMap);
      colorSensing = new ColorSensing(hardwareMap);
      spinIndexer = new SpinIndexer(hardwareMap);
      waitForStart();
      follower.startTeleopDrive(true);
      follower.update();
      while (opModeIsActive()) {
         if (gamepad1.left_bumper) {
            maxPower = 0.5;
         } else if (gamepad1.left_trigger > 0.5) {
            maxPower = 0.2;
         } else {
            maxPower = 1;
         }

         if (gamepad2.left_bumper) {
            intake.startIntake();
            if(!spinIndexer.getIsCurrentIntake()){
               spinIndexer.setPosition(spinIndexerIndex);
            }
            Artifact intakeColorDetected = colorSensing.getIntakeColorDetected();
            if (intakeColorDetected != Artifact.EMPTY && spinIndexerMovementTimer.milliseconds() > 200) {
               spinIndexer.setSectionArtifact(
                  spinIndexer.getCurrentPosition(),
                  intakeColorDetected
               );
               spinIndexerIndex += 1;
               spinIndexerIndex = spinIndexerIndex % 3;
               spinIndexer.setPosition(spinIndexerIndex);
               spinIndexerMovementTimer.reset();
            }
         } else if (gamepad2.left_trigger > 0.5) {
            intake.reverse();
         } else {
            intake.stopIntake();
         }

         if (gamepad2.right_bumper) {
            shooter.startShooter();
         } else {
            shooter.stopShooter();
         }

         if (gamepad2.aWasPressed()) {
            shooter.shoot();
            spinIndexer.setSectionArtifact(spinIndexer.getCurrentPosition(), Artifact.EMPTY);
         }

         if (gamepad2.left_bumper && gamepad2.bWasPressed()) {
            spinIndexerIndex += 1;
            spinIndexerIndex = spinIndexerIndex % 3;
            spinIndexer.setPosition(spinIndexerIndex);
         } else if (gamepad2.bWasPressed()) {
            spinIndexerIndex += 1;
            spinIndexerIndex = spinIndexerIndex % 3;
            spinIndexer.setPosition(spinIndexerIndex, 0);
         }

         follower.setMaxPower(maxPower);
         follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
         follower.update();
         telemetry.addData("velocity: ", shooter.getVelocity());
         telemetry.addData("state", spinIndexer.getSpinIndexerState());
         telemetry.addData("color: ", colorSensing.update());
      }
   }
}