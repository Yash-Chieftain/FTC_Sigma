package org.firstinspires.ftc.teamcode.TeleOp;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisms;
import org.firstinspires.ftc.teamcode.Mechanisums.Vision;
import org.firstinspires.ftc.teamcode.Utils.Artifact;

@Configurable
@TeleOp(name = "Test", group = "Linear Opmode")
public class Test extends LinearOpMode {

   public static double speed = 0.7;
   public static double rotateMultiplier = 0.5;

   Artifact selectedArtifact;
   Vision vision;

   Mechanisms mechanisms;

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

   @Override
   public void runOpMode() {

      // aprilTagYawDetector = new AprilTagYawDetector(hardwareMap, true);
      mechanisms = new Mechanisms(hardwareMap);
      vision = new Vision(hardwareMap);

      // Map motors
      frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
      frontRight = hardwareMap.get(DcMotor.class, "frontRight");
      backLeft = hardwareMap.get(DcMotor.class, "backLeft");
      backRight = hardwareMap.get(DcMotor.class, "backRight");


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

         if (gamepad2.dpad_left) {
            targetPattern = pattern[0];
         } else if (gamepad2.dpad_up) {
            targetPattern = pattern[1];
         } else if (gamepad2.dpad_right) {
            targetPattern = pattern[2];
         }


         if (gamepad2.dpad_up) {
            telemetry.addData("shoot: ", mechanisms.shoot(targetPattern));
         } else {
            if (gamepad2.left_bumper) {
               mechanisms.startIntake();
            } else {
               mechanisms.slowIntake();
            }
         }

         if(gamepad2.b){
            mechanisms.validateArtifacts();
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
      mechanisms.update();
         telemetry.addData("State:", mechanisms.getState());
         telemetry.addData("Color: ", mechanisms.getColorSensorValues());
         telemetry.addData("Shooting:", isShootingStarted ? "Started" : "Stopped");
         telemetry.update();
      }
   }
}
