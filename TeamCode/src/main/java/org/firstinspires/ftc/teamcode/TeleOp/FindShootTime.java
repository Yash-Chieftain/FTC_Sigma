package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisums.Shooter;

@Disabled
@TeleOp
public class FindShootTime extends LinearOpMode {

   Shooter shooter;
   @Override
   public void runOpMode() throws InterruptedException {
      shooter = new Shooter(hardwareMap);
      waitForStart();
      while(opModeIsActive()){
         if(gamepad2.right_bumper){
            shooter.startShooter();
         }else{
            shooter.stopShooter();
         }

         if(gamepad2.aWasPressed()){
            shooter.shoot();
         }
      }
   }
}
