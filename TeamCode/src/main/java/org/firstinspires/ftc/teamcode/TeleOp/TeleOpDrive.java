package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Mechanisums.ColorSensing;
import org.firstinspires.ftc.teamcode.Mechanisums.Intake;
import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisms;
import org.firstinspires.ftc.teamcode.Mechanisums.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisums.SpinIndexer;
import org.firstinspires.ftc.teamcode.Mechanisums.Vision;
import org.firstinspires.ftc.teamcode.Utils.Artifact;

@Configurable
@TeleOp(name = "MecanumTeleOp", group = "Linear Opmode")
public class TeleOpDrive extends LinearOpMode {

   public static double speed = 0.7;
   public static long movementSleep = 200;

   SpinIndexer spinIndexer;
   Artifact selectedArtifact;
   Intake intake;
   Shooter shooter;
   Vision vision;

   Mechanisms mechanisms;

   int index = -1;
   boolean artifactShooted = false;
   boolean isShootingStarted = false;
   ColorSensing colorSensing;
   // AprilTagYawDetector aprilTagYawDetector;
   Artifact[][] pattern = new Artifact[][]{
      {Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN},
      {Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE},
      {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE},
   };

   Artifact[] targetPattern = pattern[0];
   // Motors
   private DcMotor frontLeft, frontRight, backLeft, backRight;

   @Override
   public void runOpMode() {

      // aprilTagYawDetector = new AprilTagYawDetector(hardwareMap, true);

      intake = new Intake(hardwareMap);
      shooter = new Shooter(hardwareMap);
      vision = new Vision(hardwareMap);

      // Map motors
      frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
      frontRight = hardwareMap.get(DcMotor.class, "frontRight");
      backLeft = hardwareMap.get(DcMotor.class, "backLeft");
      backRight = hardwareMap.get(DcMotor.class, "backRight");

      spinIndexer = new SpinIndexer(hardwareMap);
      colorSensing = new ColorSensing(hardwareMap);

      // Reverse one side
      frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
      backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

      frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
      backRight.setDirection(DcMotorSimple.Direction.REVERSE);

      waitForStart();

      while (opModeIsActive()) {

         // Gamepad inputs (SWAPPED)
         double y = -gamepad1.left_stick_y;
         double x = gamepad1.left_stick_x;
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


         if (gamepad2.left_bumper && !gamepad2.x) {
            if (!spinIndexer.getIsCurrentIntake()) {
               spinIndexer.setPosition(Artifact.EMPTY, true);
            }

            Artifact intakeColorDetected = colorSensing.getIntakeColorDetected();
            if (intakeColorDetected != Artifact.EMPTY) {
               intake.slowIntake();
               spinIndexer.setSectionArtifact(
                  spinIndexer.getCurrentPosition(),
                  intakeColorDetected
               );
               spinIndexer.setPosition(Artifact.EMPTY, true);
               sleep(movementSleep);
            }

         } else if (gamepad2.left_trigger > 0.5) {
            intake.reverse();

         } else {
            intake.slowIntake();
         }

         if (gamepad2.right_bumper) {
            shooter.startShooter();
            intake.startIntake();
         } else {
            shooter.stopShooter();
         }

         if (gamepad2.right_trigger > 0.5) {
            if (!spinIndexer.setPosition(Artifact.PURPLE, false)) {
               spinIndexer.setPosition(Artifact.GREEN, false);
            }
         }

         if (Math.abs(shooter.getVelocity() - shooter.getTargetVelocity()) < 30) {
            gamepad2.rumble(100);
         }
         if (gamepad2.bWasPressed()) {
            index += 1;
            index = index % 3;
            spinIndexer.setPosition(index, 0);
         }

         if (gamepad2.xWasPressed()) {
            selectedArtifact = Artifact.PURPLE;
            if (!spinIndexer.setPosition(Artifact.PURPLE, false)) {
               colorSensing.startBlink();
            }
         }

         if (gamepad2.yWasPressed()) {
            selectedArtifact = Artifact.GREEN;
            if (!spinIndexer.setPosition(Artifact.GREEN, false)) {
               colorSensing.startBlink();
            }
         }


         if (gamepad2.dpad_left) {
            targetPattern = pattern[0];
         } else if (gamepad2.dpad_up) {
            targetPattern = pattern[1];
         } else if (gamepad2.dpad_right) {
            targetPattern = pattern[2];
         }


         if (gamepad2.aWasPressed()) {
            shooter.shoot();
            spinIndexer.setSectionArtifact(spinIndexer.getCurrentPosition(), Artifact.EMPTY);
         }


         // Mecanum math (UNCHANGED)
         double frontLeftPower = y + x + rx;
         double backLeftPower = y - x - rx;
         double frontRightPower = y - x + rx;
         double backRightPower = y + x - rx;

         double max = Math.max(
            Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
            Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
         );

         if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
         }

         frontLeft.setPower(frontLeftPower * speed);
         frontRight.setPower(backLeftPower * speed);
         backLeft.setPower(frontRightPower * speed);
         backRight.setPower(backRightPower * speed);

         colorSensing.blink();

         telemetry.addData("Color: ", colorSensing.update());
         telemetry.addData("State: ", spinIndexer.getSpinIndexerState());
         telemetry.addData("Velocity: ", shooter.getVelocity());
         telemetry.addData("Shooting:", isShootingStarted ? "Started" : "Stopped");
         telemetry.addData("isEmpty: ", spinIndexer.getArtifact(Artifact.ANY));

         // telemetry.addData("Yaw", aprilTagYawDetector.getYaw());
         telemetry.update();
      }
   }
}
