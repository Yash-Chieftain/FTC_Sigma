package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisums.Turret;

@Configurable
@TeleOp(name = "Test", group = "Linear Opmode")
public class Test extends LinearOpMode {
   Turret turret;

   @Override
   public void runOpMode() {
      turret = new Turret(hardwareMap);
      waitForStart();
      while (opModeIsActive()) {
         telemetry.addData("turret", turret.getTicks());
         telemetry.update();
      }
   }
}
