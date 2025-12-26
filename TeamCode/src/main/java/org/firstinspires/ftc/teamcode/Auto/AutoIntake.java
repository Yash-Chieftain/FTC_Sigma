package org.firstinspires.ftc.teamcode.Auto;

import android.content.om.FabricatedOverlay;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisums;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class AutoIntake extends LinearOpMode {
   public static double speed = 0.4;
   Follower follower;
   Mechanisums mechanisums;

   @Override
   public void runOpMode() throws InterruptedException {
      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
      mechanisums = new Mechanisums(hardwareMap);
      follower.setMaxPower(speed);
      follower.followPath(new Path(new BezierLine(new Pose(0, 0), new Pose(15, 0))));
      mechanisums.startIntake();
      waitForStart();
      while (opModeIsActive()) {
         mechanisums.startIntake();
         follower.update();
         telemetry.addData("Color: ", mechanisums.getColorSensorValues());
         telemetry.update();
      }
   }
}
