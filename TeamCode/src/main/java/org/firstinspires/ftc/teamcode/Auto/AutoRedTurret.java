package org.firstinspires.ftc.teamcode.Auto;

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
public class AutoRedTurret extends LinearOpMode {
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
      follower.setStartingPose(new Pose(108.947, 135.554, Math.toRadians(0)));
      follower.setMaxPower(0.9);
      mechanisms = new Mechanisms(hardwareMap);
      PathChain myPath = follower
         .pathBuilder()

         // Path1
         .addPath(
            new BezierLine(
               new Pose(108.947, 135.554),
               new Pose(90.000, 83.000)
            )
         )
         .setLinearHeadingInterpolation(
            Math.toRadians(0),
            Math.toRadians(0)
         )

         // Path2
         .addPath(
            new BezierLine(
               new Pose(90.000, 83.000),
               new Pose(103.000, 83.000)
            )
         )
         .setTangentHeadingInterpolation()

         // Path3
         .addPath(
            new BezierLine(
               new Pose(103.000, 83.000),
               new Pose(123.000, 83.000)
            )
         )
         .setTangentHeadingInterpolation()

         // Path4
         .addPath(
            new BezierLine(
               new Pose(123.000, 83.000),
               new Pose(90.000, 83.000)
            )
         )
         .setLinearHeadingInterpolation(
            Math.toRadians(0),
            Math.toRadians(0)
         )

         // Path5
         .addPath(
            new BezierCurve(
               new Pose(90.000, 83.000),
               new Pose(87.574, 58.654),
               new Pose(99.000, 59.500)
            )
         )
         .setLinearHeadingInterpolation(
            Math.toRadians(0),
            Math.toRadians(0)
         )

         // Path6
         .addPath(
            new BezierLine(
               new Pose(99.000, 59.500),
               new Pose(123.000, 59.500)
            )
         )
         .setTangentHeadingInterpolation()

         // Path7
         .addPath(
            new BezierLine(
               new Pose(123.000, 59.500),
               new Pose(90.000, 83.000)
            )
         )
         .setLinearHeadingInterpolation(
            Math.toRadians(0),
            Math.toRadians(0)
         )

         // Path8
         .addPath(
            new BezierLine(
               new Pose(90.000, 83.000),
               new Pose(105.000, 83.000)
            )
         )
         .setTangentHeadingInterpolation()

         .build();

      follower.followPath(myPath.getPath(pathState));
      mechanisms.setSpinIndexerState(new Artifact[]{
         Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE
      });
      mechanisms.pipelineSwitch(2);
      mechanisms.setTurretTicks(283);
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
            } else if (pathState == 3|| pathState == 6|| pathState == 9) {
               follower.followPath(new PathChain(PedroUtils.getPath(follower.getPose(), myPath.getPath(pathState).endPose())));
            } else {
               follower.followPath(myPath.getPath(pathState));
            }
         } else {
            if(pathState == 0){
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
               if (follower.getCurrentTValue() < 0.4 && !(mechanisms.getNoOfArtifacts() < 3)) {
                  mechanisms.startIntake();
               } else {
                  mechanisms.readyToShoot(targetMotif);
               }
               mechanisms.rampUpShooter();
            }
         }
         telemetry.addData("State: ", mechanisms.getState());
         telemetry.update();
         mechanisms.update();
         follower.update();
      }
   }
}
