package org.firstinspires.ftc.teamcode.Mechanisums;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Configurable
public class Vision {
   public static double kP = 0.04;
   public static double kD = 0.004;
   public static double maxPower = 0.45;
   public static double dist_tolerance = 2;
   private static double limelightAngleMounted = 12;
   private static double limelightLensHeight = 11.5;
   private static double goalHeightInches = 29.5;
   Limelight3A limelight;
   private double previousTx = 0;
   private double previousTime = 0;

   public Vision(HardwareMap hardwareMap) {
      limelight = hardwareMap.get(Limelight3A.class, "limelight");
      limelight.pipelineSwitch(0);
      limelight.start();
   }

   public double getDistance() {
      LLResult result = limelight.getLatestResult();
      if (result != null && result.isValid()) {
         double ty = result.getTy();
         double angleToGoalDegrees = limelightAngleMounted + ty;
         double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);


         return (goalHeightInches - limelightLensHeight) / Math.tan(angleToGoalRadians);

      } else {
         return -1;
      }
   }

   public double alignTurnValue(double manualTurn) {
      LLResult result = limelight.getLatestResult();

      double turnPower = manualTurn; // default = driver control

      if (result.isValid()) {
         double tx = result.getTx();

         double currentTime = System.currentTimeMillis() / 1000.0;
         double deltaTime = currentTime - previousTime;
         double derivative = 0;

         if (deltaTime > 0) {
            derivative = (tx - previousTx) / deltaTime;
         }
         // Add automatic correction to driver’s turn
         double autoTurn = (kP * tx) + (kD * derivative);
         autoTurn = Math.max(-maxPower, Math.min(autoTurn, maxPower));

         if (Math.abs(tx) < dist_tolerance) {
            autoTurn = 0; // stop turning when aligned
         }

         turnPower = autoTurn;

         previousTx = tx;
         previousTime = currentTime;
      }
      return turnPower;
   }

   public double getTx() {
      return previousTx;
   }

}