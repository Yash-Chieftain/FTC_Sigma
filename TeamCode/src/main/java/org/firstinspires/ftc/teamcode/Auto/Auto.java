package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisums;
import org.firstinspires.ftc.teamcode.Utils.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class Auto extends LinearOpMode {
   public static double intakeDriveSpeed = 0.3;
   Follower follower;
   int pathState = 0;
   Mechanisums mechanisums;
   Artifact[] targetMotif = new Artifact[]{
      Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE
   };

   @Override
   public void runOpMode() throws InterruptedException {
      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(new Pose(34.298, 135.777, Math.toRadians(180)));
      follower.setMaxPower(0.8);
      mechanisums = new Mechanisums(hardwareMap);
      mechanisums.setSpinIndexerState(new Artifact[]{
         Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE
      });
      PathChain myPath = follower
         .pathBuilder()
         .addPath(
            new BezierLine(new Pose(34.298, 135.777), new Pose(55.000, 83.500))
         )
         .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
         .addPath(
            new BezierLine(new Pose(55.000, 83.500), new Pose(42.000, 83.500))
         )
         .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
         .addPath(
            new BezierLine(new Pose(42.000, 83.500), new Pose(16.000, 83.500))
         )
         .setTangentHeadingInterpolation()

         .addPath(
            new BezierCurve(
               new Pose(16.000, 83.500),
               new Pose(33.094, 76.813),
               new Pose(15.000, 72.000)
            )
         )
         .setConstantHeadingInterpolation(Math.toRadians(180))
         .addPath(
            new BezierLine(new Pose(15.000, 72.000), new Pose(55.000, 83.500))
         )
         .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
         .build();


      follower.followPath(myPath.getPath(pathState));

      waitForStart();
      while (opModeIsActive()) {
         if (!follower.isBusy()) {
            if (pathState == 0 || pathState == 4) {
               while (!mechanisums.shoot(targetMotif)) {
                  mechanisums.update();
               }
               mechanisums.startIntake();
            } else if (pathState == 2) {
               mechanisums.slowIntake();
            }
            pathState++;
            if (pathState == 1) {
               follower.followPath(new PathChain(myPath.getPath(pathState)), false);
            } else if (pathState == 2) {
               follower.followPath(new PathChain(myPath.getPath(pathState)), intakeDriveSpeed, true);
            } else {
               follower.followPath(myPath.getPath(pathState));
            }
         } else {
            if (pathState == 1 || pathState == 2) {
               mechanisums.startIntake();
            }
         }
         telemetry.addData("State: ", mechanisums.getState());
         mechanisums.update();
         follower.update();
      }
   }
}