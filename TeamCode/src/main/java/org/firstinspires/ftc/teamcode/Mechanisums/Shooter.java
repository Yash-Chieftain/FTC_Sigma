package org.firstinspires.ftc.teamcode.Mechanisums;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Wait;

@Configurable
public class Shooter {
   public static double velocity = 950;
   public static double kp = 250;
   public static double ki = 0;
   public static double kd = 0;
   public static double kf = 19;
   public static long shootDelay = 100;
   public static double shootPosition =  0.700;
   DcMotorEx leftMotor, rightMotor;
   Servo bootKicker1, bootKicker2;
   public Shooter(HardwareMap hardwareMap) {
      leftMotor = hardwareMap.get(DcMotorEx.class, "leftshoot");
      rightMotor = hardwareMap.get(DcMotorEx.class, "rightshoot");
      leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
//      bootKicker1 = hardwareMap.get(Servo.class, "bootKicker1");
//      bootKicker1.setPosition(1);
      bootKicker2 = hardwareMap.get(Servo.class, "bootKicker2");
      bootKicker2.setPosition(0);
   }

   public void startShooter() {
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

      leftMotor.setVelocity(velocity);
      rightMotor.setVelocity(velocity);
   }

   public void stopShooter() {
      leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      leftMotor.setPower(0);
      rightMotor.setPower(0);
   }

   public double getVelocity() {
      return leftMotor.getVelocity();
   }

   public double getTargetVelocity(){
      return velocity;
   }

   public boolean isVelocityReached(){
      return Math.abs(getVelocity() - velocity) < 30;
   }


   public void shoot() {
      if (this.isVelocityReached()) {
//         bootKicker1.setPosition(0.532);
         bootKicker2.setPosition(shootPosition);
         Wait.mySleep(shootDelay);
//         bootKicker1.setPosition(1);
         bootKicker2.setPosition(0);
         Wait.mySleep(shootDelay/3);
      }
   }
}
