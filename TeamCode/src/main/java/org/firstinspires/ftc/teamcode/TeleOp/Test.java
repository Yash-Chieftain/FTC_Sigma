package org.firstinspires.ftc.teamcode.TeleOp;


import androidx.core.view.TintableBackgroundView;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisms;
import org.firstinspires.ftc.teamcode.Mechanisums.Vision;
import org.firstinspires.ftc.teamcode.Utils.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Test", group = "Linear Opmode")
public class Test extends LinearOpMode {

   public static double speed = 0.7;

   Artifact selectedArtifact;
   Vision vision;

   Mechanisms mechanisums;

   int index = -1;
   boolean artifactShooted = false;
   boolean isShootingStarted = false;
   Artifact[][] pattern = new Artifact[][]{
      {Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE},
      {Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN},
      {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE},

   };

   Artifact[] targetPattern = pattern[0];
   // Motors
   private DcMotor frontLeft, frontRight, backLeft, backRight;
Follower follower;
   @Override
   public void runOpMode() {

      // aprilTagYawDetector = new AprilTagYawDetector(hardwareMap, true);
      mechanisums = new Mechanisms(hardwareMap);
      follower = Constants.createFollower(hardwareMap);
      waitForStart();
follower.startTeleopDrive(true);
follower.update();
      while (opModeIsActive()) {

         // Gamepad inputs (SWAPPED)
         double y = -gamepad1.left_stick_y;
         double x = -gamepad1.left_stick_x;
         double rx = gamepad1.right_stick_x;

         if (gamepad1.b) {
            rx = vision.alignTurnValue(rx);
         }

         if (gamepad1.left_trigger > 0.5) {
            speed = 0.25;
         } else if (gamepad1.right_bumper) {
            speed = 0.5;
         } else {
            speed = 1;
         }

         if (gamepad2.dpad_left) {
            targetPattern = pattern[0];
         } else if (gamepad2.dpad_up) {
            targetPattern = pattern[1];
         } else if (gamepad2.dpad_right) {
            targetPattern = pattern[2];
         }


         if (gamepad2.dpad_up) {
            telemetry.addData("shoot: ", mechanisums.shoot(targetPattern));
         } else {
            if (gamepad2.left_bumper) {
               mechanisums.startIntake();
            } else {
               mechanisums.slowIntake();
            }
         }



      mechanisums.update();
         telemetry.addData("State:", mechanisums.getState());
         telemetry.addData("Color: ", mechanisums.getColorSensorValues());
         telemetry.addData("Shooting:", isShootingStarted ? "Started" : "Stopped");
         telemetry.update();
         follower.setTeleOpDrive(y,x, rx);
         follower.update();
      }
   }
}
