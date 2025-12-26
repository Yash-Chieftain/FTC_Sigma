//package org.firstinspires.ftc.teamcode;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@Configurable
//@TeleOp(name = "TeleOp Main", group = "Main")
//public class TeleOpMain extends OpMode {
//
//   // Drivetrain motors
//   private DcMotor frontLeft, frontRight, backLeft, backRight;
//   public static double intakePower = 0.6;
//   public static double intakeShootPower = 0.6;
//   public static double speed = 0.5;
//   Shooter shooter;
//   DcMotor intake;
//   CRServo servo;
//
//   @Override
//   public void init() {
//   intake = hardwareMap.get(DcMotor.class, "intake");
//   servo= hardwareMap.get(CRServo.class, "Intake Support");
//      shooter = new Shooter(hardwareMap);
//      // Map motors
//      frontLeft  = hardwareMap.get(DcMotor.class, "LF");
//      frontRight = hardwareMap.get(DcMotor.class, "RF");
//      backLeft   = hardwareMap.get(DcMotor.class, "LR");
//      backRight  = hardwareMap.get(DcMotor.class, "RR");
//
//      // Reverse left side (common for mecanum)
//
//
//      telemetry.addLine("Initialized!");
//   }
//
//   @Override
//   public void loop() {
//
//      // --------------------------
//      // 🔹 MECANUM DRIVE CONTROL
//      // --------------------------
//
//      double y = -gamepad1.left_stick_y;     // forward/back
//      double x = gamepad1.left_stick_x;      // strafe
//      double rx = gamepad1.right_stick_x;    // rotate
//
//      double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//      double frontLeftPower  = (y + x + rx) / denominator;
//      double backLeftPower   = (y - x + rx) / denominator;
//      double frontRightPower = (y - x - rx) / denominator;
//      double backRightPower  = (y + x - rx) / denominator;
//
//      frontLeft.setPower(frontLeftPower * speed);
//      backLeft.setPower(backLeftPower * speed);
//      frontRight.setPower(frontRightPower* speed);
//      backRight.setPower(backRightPower * speed);
//
//      if(gamepad1.right_bumper){
//         shooter.startShooter();
//      }else{
//         shooter.stopShooter();
//      }
//
//      if(gamepad1.left_bumper){
//         intake.setPower(intakePower);
//         servo.setPower(1);
//      }else if(gamepad1.a){
//         intake.setPower(intakeShootPower);
//         servo.setPower(-1);
//      }else{
//         intake.setPower(0);
//         servo.setPower(0);
//      }
//
//
//
//      // --------------------------
//      // 🔹 TELEMETRY
//      // --------------------------
//
//      telemetry.addData("FL", frontLeftPower);
//      telemetry.addData("FR", frontRightPower);
//      telemetry.addData("BL", backLeftPower);
//      telemetry.addData("BR", backRightPower);
//
//      telemetry.addData("LX", gamepad1.left_stick_x);
//      telemetry.addData("LY", gamepad1.left_stick_y);
//      telemetry.addData("RX", gamepad1.right_stick_x);
//
//      telemetry.addData("Left Shooter: ", shooter.getVelocity()[0]);
//      telemetry.addData("Right Shooter: ", shooter.getVelocity()[1]);
//      telemetry.update();
//   }
//}
