package org.firstinspires.ftc.teamcode.Mechanisums;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Turret {

   // ================= CONFIG =================

   /**
    * Max mechanical rotation (degrees)
    */
   public static double MAX_DEGREES = 300;

   /**
    * Encoder ticks per motor revolution
    */
   public static double TICKS_PER_REV = 537.6; // GoBILDA 5202 (change if needed)

   /**
    * Gear ratio (motor : turret)
    */
   public static double GEAR_RATIO = 1.0; // change if geared

   /**
    * Motor power while moving
    */
   public static double MOTOR_POWER = 0.3;

   // ==========================================

   private DcMotorEx turretMotor;
   private double targetDegrees = 0;
   private static double kp = 0.09;
   private static double maxPower = 0.5;
   public Turret(HardwareMap hardwareMap) {
      turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

//      turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
      turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
   }

   /**
    * Call every loop
    */
   public void update() {
      int targetTicks = degreesToTicks(targetDegrees);

      turretMotor.setTargetPosition(targetTicks);
      turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
      turretMotor.setPower(MOTOR_POWER);
   }

   /**
    * Set desired turret angle
    */
   public void setTargetDegrees(double deg) {
      targetDegrees = wrapDegrees(deg);
   }

   /**
    * Convert degrees → encoder ticks
    */
   public int degreesToTicks(double deg) {
      return (int) ((deg / 360.0) * TICKS_PER_REV * GEAR_RATIO);
   }

   /**
    * Convert encoder ticks → degrees
    */
   public double getCurrentDegrees() {
      int ticks = turretMotor.getCurrentPosition();
      return (ticks / (TICKS_PER_REV * GEAR_RATIO)) * 360.0;
   }

   public double getTicks() {
      return turretMotor.getCurrentPosition();
   }

   public void alignLimeLight(double tx) {
      double power = tx * kp;
      power = Math.max(-maxPower, Math.min(maxPower, power));
      turretMotor.setPower(-power);
   }


   /**
    * Wrap angle to 0–MAX_DEGREES
    */
   public double wrapDegrees(double deg) {
      deg %= MAX_DEGREES;
      if (deg < 0) deg += MAX_DEGREES;
      return deg;
   }

   /**
    * Stop motor
    */
   public void stop() {
      turretMotor.setPower(0);
   }
}
