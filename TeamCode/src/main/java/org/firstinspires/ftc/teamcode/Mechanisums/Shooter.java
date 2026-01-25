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
   public static double kp = 250;
   public static double ki = 0;
   public static double kd = 10;
   public static double kf = 20.2;
   public static long shootDelay = 150;
   public static double shootPosition = 0.700;
   public static double longshootpos = 0.9;
   public static double shortshootpos = 0.08;
   public static double divideShootTime = 1;
   public static double longvelocity = 1430;
   public static double shortvelocity = 1150;
   public static double testVelocity = 1200;

   // Linear fit constants
   public static double HOOD_M = 0.00582003344;
   public static double HOOD_B = -0.3886168111;

   public static double VELOCITY_M = 3.089375285;
   public static double VELOCITY_B = 980.3716370;
   public double velocity = 1200, hood = 0;
   DcMotorEx leftMotor, rightMotor;
   Servo leftHood, rightHood;

   Servo bootKicker1, bootKicker2;


   public Shooter(HardwareMap hardwareMap) {
      leftMotor = hardwareMap.get(DcMotorEx.class, "leftshoot");
      rightMotor = hardwareMap.get(DcMotorEx.class, "rightshoot");
      leftHood = hardwareMap.get(Servo.class, "leftHood");
      rightHood = hardwareMap.get(Servo.class, "rightHood");
      rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      leftHood.setDirection(Servo.Direction.REVERSE);
      setHoodPosition(0.1);
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
      bootKicker1 = hardwareMap.get(Servo.class, "bootKicker1");
      bootKicker1.setDirection(Servo.Direction.REVERSE);
      bootKicker1.setPosition(0.01);
      bootKicker2 = hardwareMap.get(Servo.class, "bootKicker2");
      bootKicker2.setPosition(0.01);
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
      setHoodPosition(hood);
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

   public double getTargetVelocity() {
      return velocity;
   }

   public double getHoodPosition() {
      return hood;
   }

   public void setHoodPosition(double position) {
      leftHood.setPosition(position);
      rightHood.setPosition(position);
   }

   public boolean isVelocityReached() {
      return Math.abs(getVelocity() - velocity) < 20;

   }

   public boolean shoot() {
      if (this.isVelocityReached()) {
         bootKicker1.setPosition(shootPosition);
         bootKicker2.setPosition(shootPosition);
         Wait.mySleep(shootDelay);
         bootKicker1.setPosition(0.01);
         bootKicker2.setPosition(0.01);
         Wait.mySleep((long) (shootDelay / divideShootTime));
         return true;
      }
      return false;
   }

   public void setShooter(double distance) {
      if (distance<33) {
         hood=shortshootpos;
         velocity=shortvelocity;
         return;
      } else if (distance>110) {
         hood=longshootpos;
         velocity=longvelocity;
         return;

      }
      hood = getHoodAngleByDistance(distance);
      velocity = getVelocityByDistance(distance);
   }

   public void startLongshooter() {
      velocity = longvelocity;
      startShooter();
      setHoodPosition(longshootpos);
   }

   public void startShortShooter() {
      velocity = shortvelocity;
      startShooter();
      setHoodPosition(shortshootpos);

   }

   public double getVelocityByDistance(double distance) {
      return (distance - 33) * 5.55 + 950;
   }

   public double getHoodAngleByDistance(double distance) {
      return (distance - 33) * 0.0038 + 50;
   }

   public double[] getShooterValuesPiecewise(double d) {

      double hood, velocity;
      if (d <= 86) {
         hood = 0.0;
         velocity = 3.4091 * d + 956.82;

      } else if (d <= 101) {
         hood = 0.003333 * d - 0.2867;
         velocity = 3.3333 * d + 963.33;

      } else if (d <= 162) {
         hood = 0.008115 * d - 0.7696;
         velocity = 2.4590 * d + 1081.7;

      } else {
         hood = 0.02667 * d - 3.782;
         velocity = 8.3333 * d + 100.0;
      }

      // Safety clamps
      hood = Math.max(0.0, Math.min(1.0, hood));
      velocity = Math.max(900, Math.min(1600, velocity));

      return new double[]{hood, velocity};
   }

}

