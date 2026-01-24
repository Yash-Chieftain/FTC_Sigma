package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisms;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Utils.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@TeleOp
public class RedTeleOp extends LinearOpMode {
   public static double kp = 0.02;
   double maxPower = RobotConstants.TeleOp.maxPower;
   double turnSensitivity = RobotConstants.TeleOp.turnSensitivity;
   TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
   Mechanisms mechanisms;
   Follower follower;
   int spinIndexerIndex = 0;
   int numberOfArtifacts = 0;
   double heading = 0;
   boolean isStrafeStarted = false;
   double hoodPosition = 0.5;
   boolean isPark = true;
   int path;
   Artifact[] pattern = new Artifact[]{Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};

   @Override
   public void runOpMode() throws InterruptedException {

      double y = 0, x = 0, rx = 0;
      follower = Constants.createFollower(hardwareMap);
      mechanisms = new Mechanisms(hardwareMap);
      mechanisms.pipelineSwitch(RobotConstants.Mechanisms.Vision.redAllianceGoalPipelineIndex);
      waitForStart();
      // Above is INIT

      follower.startTeleopDrive(true);
      follower.update();
      while (opModeIsActive()) {
         kp = RobotConstants.TeleOp.kp;
         y = -gamepad1.left_stick_y;
         x = -gamepad1.left_stick_x;
         rx = -gamepad1.right_stick_x * turnSensitivity;
         if (gamepad1.left_bumper) {
            maxPower = 0.75;
         } else if (gamepad1.left_trigger > 0.5) {
            maxPower = 0.4;
         } else if (gamepad1.right_trigger > 0.5) {
            y = y * -1;
            x = x * -1;
            rx = rx * -1;
         } else if (gamepad1.right_bumper) {
            maxPower = 0.4;
         }else {
            maxPower = 1;
         }

         if (gamepad2.left_bumper && gamepad2.bWasPressed()) {
            spinIndexerIndex += 1;
            spinIndexerIndex = spinIndexerIndex % 3;
            mechanisms.setSpinIndexerIntakePosition(spinIndexerIndex);
         } else if (gamepad2.bWasPressed()) {
            spinIndexerIndex += 1;
            spinIndexerIndex = spinIndexerIndex % 3;
            mechanisms.setSpinIndexerShootingPosition(spinIndexerIndex);
         }

         if (gamepad1.b) {
            rx = -mechanisms.getTurnValue();
         } else if (Math.abs(rx) < 0.01 && (Math.abs(x) > 0.01 || Math.abs(y) > 0.01)) {
            if (!isStrafeStarted) {
               heading = follower.getHeading();
               isStrafeStarted = true;
            }
            double error = heading - follower.getHeading();
            rx = error * kp;

         } else {
            isStrafeStarted = false;
         }
         while (gamepad1.dpad_up){
         }

         if (gamepad1.aWasPressed()){
            follower.breakFollowing();
         }

         if (gamepad1.y) {
            mechanisms.park();
         }
         if (gamepad1.dpad_down) {
            mechanisms.unPark();
         }


//         int e = 0;
//         while (e < 3) {
//            int a = numberOfArtifacts + e;
//            System.out.println("Length" + a);
//            System.out.println("Current contents" + numberOfArtifacts);
//            System.out.println("Current contents" + pattern);
//            int b = pattern.length;
//            System.out.println("Length" + b);
//            int result = a % b;
//            System.out.println("result" + result);
//            Artifact value = pattern[result];
//            System.out.println("Shoot" + value);
//            e++;
//         }
//         System.out.println(numberOfArtifacts);
         //FAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHHHHHHHH

         if (gamepad2.left_bumper) {
            mechanisms.startIntake();
         } else if (gamepad2.left_trigger > 0.5) {
            mechanisms.reverseIntake();
         } else {
            mechanisms.stopIntake();
         }


         if (gamepad2.right_bumper) {
            mechanisms.startShooter();
         } else {
            mechanisms.stopShooter();
         }
         if (gamepad2.dpadUpWasPressed()) {
            numberOfArtifacts += 1;
         } else if (gamepad2.dpadDownWasPressed()) {
            numberOfArtifacts -= 1;
         }

         if (gamepad2.aWasPressed()) {
            mechanisms.shoot();
         }
         if (gamepad2.x) {
            mechanisms.shoot(pattern);
         }


         if (gamepad2.yWasPressed()) {
            mechanisms.setSpinIndexerState(new Artifact[]{Artifact.EMPTY, Artifact.EMPTY, Artifact.EMPTY});
         }

         follower.setMaxPower(maxPower);
         follower.setTeleOpDrive(y, x, rx, true);
         follower.update();
         telemetryManager.addData("Velocity", mechanisms.getShooterVelocity());
         telemetryManager.addData("Color", mechanisms.getColorSensorValues());
         telemetryManager.addData("State", mechanisms.getState());
         telemetryManager.addData("getTx", mechanisms.getTurnValue());
         telemetryManager.addData("Sequence: ", mechanisms.getShootingSequenceString(pattern));
         telemetryManager.addData("Target Velo: ", mechanisms.getTargetVelocity());
         telemetryManager.addData("hoodposition", hoodPosition);
         telemetryManager.update(telemetry);
         mechanisms.update();

      }
   }
}