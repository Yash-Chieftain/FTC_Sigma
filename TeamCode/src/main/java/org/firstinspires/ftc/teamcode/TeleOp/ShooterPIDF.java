package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Configurable
@TeleOp
public class ShooterPIDF extends LinearOpMode {

   public static double velocity = 1500;
   public static double kp = 200;
   public static double ki = 1;
   public static double kd = 10;
   public static double kf = 11.7;

   DcMotorEx leftMotor, rightMotor;
   ElapsedTime timer = new ElapsedTime();

   double timeToVelocity = -1;
   boolean wasRBPressed = false;

   public void setVelocity(double velocity){
      leftMotor.setVelocity(velocity);
      rightMotor.setVelocity(velocity);
   }

   @Override
   public void runOpMode() throws InterruptedException {

      leftMotor = hardwareMap.get(DcMotorEx.class, "leftShoot");
      rightMotor = hardwareMap.get(DcMotorEx.class, "rightShoot");
      leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      leftMotor.setPIDFCoefficients(
         DcMotor.RunMode.RUN_USING_ENCODER,
         new PIDFCoefficients(kp, ki, kd, kf)
      );
      rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rightMotor.setPIDFCoefficients(
         DcMotor.RunMode.RUN_USING_ENCODER,
         new PIDFCoefficients(kp, ki, kd, kf)
      );

      rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

      waitForStart();

      while (opModeIsActive()) {

         boolean rb = gamepad1.right_bumper;

         // Detect RIGHT BUMPER press (rising edge)
         if (rb && !wasRBPressed) {
            timer.reset();
            timeToVelocity = -1;
         }

         if (rb) {
this.setVelocity(velocity);
            // Capture time once velocity is reached
            if (Math.abs(leftMotor.getVelocity() - velocity) < 30 && timeToVelocity < 0) {
               timeToVelocity = timer.milliseconds();
            }
         } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            timeToVelocity = -1;
         }

         wasRBPressed = rb;
         leftMotor.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            new PIDFCoefficients(kp, ki, kd, kf)
         );
         rightMotor.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            new PIDFCoefficients(kp, ki, kd, kf)
         );
         telemetry.addData("Target Velocity", velocity);
         telemetry.addData("Current Velocity", leftMotor.getVelocity());
         telemetry.addData("Time to Reach (ms)", timeToVelocity < 0 ? "Waiting..." : timeToVelocity);
         telemetry.update();
      }
   }
}
