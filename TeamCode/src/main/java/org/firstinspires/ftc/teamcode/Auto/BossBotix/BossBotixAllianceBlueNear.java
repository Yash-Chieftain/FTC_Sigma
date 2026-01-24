package org.firstinspires.ftc.teamcode.Auto.BossBotix;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisms;
import org.firstinspires.ftc.teamcode.Utils.Artifact;
import org.firstinspires.ftc.teamcode.Utils.PedroUtils;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class BossBotixAllianceBlueNear extends LinearOpMode {
   public static double intakeDriveSpeed = 0.29;
   Follower follower;
   int pathState = 0;
   Mechanisms mechanisms;
   Artifact[] targetMotif = new Artifact[]{
      Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE
   };


   @Override
   public void runOpMode() throws InterruptedException {

      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(new Pose(34.298, 135.777, Math.toRadians(180)));
      follower.setMaxPower(0.95);
      mechanisms = new Mechanisms(hardwareMap);
      mechanisms.setSpinIndexerState(new Artifact[]{
         Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE
      });

      PathChain myPath = follower
         .pathBuilder()
         // Path1
         .addPath(
            new BezierLine(
               new Pose(34.340, 135.955),
               new Pose(54.000, 83.500)
            )
         )
         .setConstantHeadingInterpolation(Math.toRadians(180))

         // Path2
         .addPath(
            new BezierLine(
               new Pose(50.000, 83.500),
               new Pose(42.000, 83.500)
            )
         )
         .setConstantHeadingInterpolation(Math.toRadians(180))

         // Path3
         .addPath(
            new BezierLine(
               new Pose(42.000, 83.500),
               new Pose(20.000, 83.500)
            )
         )
         .setTangentHeadingInterpolation()

         // Path4
         .addPath(
            new BezierLine(
               new Pose(20.000, 83.500),
               new Pose(50.000, 83.500)
            )
         )
         .setConstantHeadingInterpolation(Math.toRadians(180))

         // Path5
         .addPath(
            new BezierCurve(
               new Pose(50.000, 83.500),
               new Pose(65.100, 57.552),
               new Pose(42.000, 60.200)
            )
         )
         .setConstantHeadingInterpolation(Math.toRadians(180))

         // Path6
         .addPath(
            new BezierLine(
               new Pose(42.000, 60.200),
               new Pose(20.000, 60.200)
            )
         )
         .setTangentHeadingInterpolation()

         // Path7
         .addPath(
            new BezierLine(
               new Pose(20.000, 60.200),
               new Pose(50.000, 82.500)
            )
         )
         .setConstantHeadingInterpolation(Math.toRadians(180))
         .build();

      follower.followPath(myPath.getPath(pathState));
      mechanisms.setTurretTicks(-283);
      waitForStart();
      while (opModeIsActive()) {
         if (!follower.isBusy()) {
            if (pathState == 0 || pathState == 3 || pathState == 6 || pathState == 9) {
               while (mechanisms.getTurnValue() != 0) {
                  mechanisms.update();
                  mechanisms.startShooter();
               }
               while (!mechanisms.shoot(targetMotif)) {
                  mechanisms.update();
               }
               mechanisms.startIntake();
            }
            pathState++;
            if (pathState == 1 || pathState == 4 || pathState == 7) {
               follower.followPath(new PathChain(myPath.getPath(pathState)), false);
            } else if (pathState == 2 || pathState == 5 || pathState == 8) {
               follower.followPath(new PathChain(myPath.getPath(pathState)), intakeDriveSpeed, true);
            } else if (pathState == 3 || pathState == 6 || pathState == 9) {
               follower.followPath(new PathChain(PedroUtils.getPath(follower.getPose(), myPath.getPath(pathState).endPose())));
            } else {
               follower.followPath(myPath.getPath(pathState));
            }
         } else {
            if (pathState == 0) {
               mechanisms.readyToShoot(targetMotif);
               mechanisms.rampUpShooter();
            }
            if (pathState == 2 || pathState == 5 || pathState == 8) {
               mechanisms.startIntake();
               if (mechanisms.getNoOfArtifacts() >= 3) {
                  follower.breakFollowing();
               }
            }

            if (pathState == 3 || pathState == 6 || pathState == 9) {
               if (follower.getCurrentTValue() < 0.4) {
                  if (mechanisms.getNoOfArtifacts() < 3) {
                     mechanisms.startIntake();
                  }else{
                     mechanisms.reverseIntake();
                  }

               } else {
                  mechanisms.readyToShoot(targetMotif);
               }
               mechanisms.rampUpShooter();
            }
         }
         telemetry.addData("State: ", mechanisms.getState());
         telemetry.addData("Pose", follower.getPose().toString());
         telemetry.update();
         mechanisms.update();
         follower.update();
      }
   }
}

