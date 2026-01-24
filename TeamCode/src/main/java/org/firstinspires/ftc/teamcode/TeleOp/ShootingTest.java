package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisms;

@TeleOp
public class ShootingTest extends LinearOpMode {
   Mechanisms mechanisms;
   int spinIndexerIndex = 0;
   TelemetryManager telemetryManager;
   @Override
   public void runOpMode() throws InterruptedException {
      telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
      double hoodPos = 0;
      mechanisms = new Mechanisms(hardwareMap);
      waitForStart();
      while (opModeIsActive()){
         if(gamepad1.right_bumper){
            mechanisms.startShooter();
         }else{
            mechanisms.stopShooter();
         }

         if (gamepad1.left_bumper && gamepad1.bWasPressed()) {
            spinIndexerIndex += 1;
            spinIndexerIndex = spinIndexerIndex % 3;
            mechanisms.setSpinIndexerIntakePosition(spinIndexerIndex);
         } else if (gamepad1.bWasPressed()) {
            spinIndexerIndex += 1;
            spinIndexerIndex = spinIndexerIndex % 3;
            mechanisms.setSpinIndexerShootingPosition(spinIndexerIndex);
         }


         if (gamepad1.left_bumper) {
            mechanisms.startIntake();
         } else if (gamepad1.left_trigger > 0.5) {
            mechanisms.reverseIntake();
         } else {
            mechanisms.stopIntake();
         }

         if(gamepad1.aWasPressed()){
            mechanisms.shoot();
         }

         if(gamepad1.dpad_up){
            hoodPos += 0.005;
         }else if(gamepad1.dpad_down){
            hoodPos -= 0.005;
         }
         mechanisms.setShooterhood(hoodPos);
         telemetryManager.addData("velocity", mechanisms.getShooterVelocity());
         telemetryManager.addData("Distance", mechanisms.getDistance());
         telemetryManager.addData("Hood", hoodPos);
         telemetryManager.addData("Ty", mechanisms.getTy());
         telemetryManager.update(telemetry);
         mechanisms.update();
      }
   }
}
