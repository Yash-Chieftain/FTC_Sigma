package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class AutoIntake extends LinearOpMode {
   public static double speed = 0.4;
   Follower follower;
   Mechanisms mechanisms;

   @Override
   public void runOpMode() throws InterruptedException {
      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
      mechanisms = new Mechanisms(hardwareMap);
      follower.setMaxPower(speed);
      follower.followPath(new Path(new BezierLine(new Pose(0, 0), new Pose(15, 0))));
      mechanisms.startIntake();
      waitForStart();
      while (opModeIsActive()) {
         mechanisms.startIntake();
         follower.update();
         telemetry.addData("Color: ", mechanisms.getColorSensorValues());
         telemetry.update();
      }
   }
}
