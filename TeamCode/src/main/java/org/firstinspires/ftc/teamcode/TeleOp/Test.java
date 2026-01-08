package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisms;
import org.firstinspires.ftc.teamcode.Mechanisums.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisums.SpinIndexer;
import org.firstinspires.ftc.teamcode.Utils.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class Test extends LinearOpMode {
   Mechanisms mechanisms;


   @Override
   public void runOpMode() throws InterruptedException {
      mechanisms = new Mechanisms(hardwareMap);
      waitForStart();
      while (opModeIsActive()) {
         if (gamepad1.left_bumper) {
            mechanisms.startIntake();
         }else{
            mechanisms.stopIntake();
         }
         if (gamepad1.right_bumper) {
            mechanisms.startShooter();
         }else{
            mechanisms.stopShooter();
         }
         if (gamepad1.a) {
            mechanisms.shoot(new Artifact[]{Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE});
         }
         telemetry.addData("Velocity", mechanisms.getShooterVelocity());
         telemetry.addData("State",mechanisms.getState());
         telemetry.addData("Color", mechanisms.getColorSensorValues());
         telemetry.addData("getTx", mechanisms.getTurnValue());
         telemetry.update();
         mechanisms.update();
      }


   }
}