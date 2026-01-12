package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisms;
import org.firstinspires.ftc.teamcode.Utils.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class VrishaanAuto extends LinearOpMode {
   public static double intakeDriveSpeed = 0.25;
   Follower follower;
   int pathState = 0;
   Mechanisms mechanisms;
   Artifact[] targetMotif = new Artifact[]{
      Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE
   };

   private DcMotor frontLeft, frontRight, backLeft, backRight;

   @Override
   public void runOpMode() throws InterruptedException {

      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(new Pose(34.298, 135.777, Math.toRadians(180)));
      follower.setMaxPower(0.8);
      mechanisms = new Mechanisms(hardwareMap);
      mechanisms.setSpinIndexerState(new Artifact[]{
         Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE
      });


      PathChain myPath = follower
         .pathBuilder()
         // Path1

         .addPath(
            new BezierLine(
               new Pose(33.740, 136.832),
               new Pose(60.531, 84.460)
            )
         )
         .setLinearHeadingInterpolation(
            Math.toRadians(180),
            Math.toRadians(135)
         )

         // Path2
         .addPath(
            new BezierLine(
               new Pose(60.531, 84.460),
               new Pose(43.390, 84.033)
            )
         )
         .setLinearHeadingInterpolation(
            Math.toRadians(135),
            Math.toRadians(180)
         )

         // Path3
         .addPath(
            new BezierLine(
               new Pose(43.390, 84.033),
               new Pose(13.782, 83.878)
            )
         )
         .setTangentHeadingInterpolation()
         // Path4
         .addPath(
            new BezierCurve(
               new Pose(13.782, 83.878),
               new Pose(48.386, 75.980),
               new Pose(14.073, 73.871)
            )
         )
         .setLinearHeadingInterpolation(
            Math.toRadians(180),
            Math.toRadians(360)
         )

         // Path5
         .addPath(
            new BezierLine(
               new Pose(14.073, 73.871),

               new Pose(60.070, 83.825)
            )
         )
         .setLinearHeadingInterpolation(
            Math.toRadians(360),
            Math.toRadians(135)
         )

         // Path6
         .addPath(
            new BezierLine(
               new Pose(60.070, 83.825),

               new Pose(46.635, 60.550)
            )
         )
         .setLinearHeadingInterpolation(
            Math.toRadians(135),
            Math.toRadians(180)
         )

         // Path7
         .addPath(
            new BezierLine(
               new Pose(46.635, 60.550),

               new Pose(18.073, 60.231)
            )
         )
         .setTangentHeadingInterpolation()

         // Path8
         .addPath(
            new BezierLine(
               new Pose(18.073, 60.231),

               new Pose(60.941, 84.135)
            )
         )
         .setLinearHeadingInterpolation(
            Math.toRadians(180),
            Math.toRadians(135)
         )

         // Path9
         .addPath(
            new BezierLine(
               new Pose(60.941, 84.135),
               new Pose(45.964, 35.460)
            )
         )
         .setLinearHeadingInterpolation(
            Math.toRadians(135),
            Math.toRadians(180)
         )

         // Path10
         .addPath(
            new BezierLine(
               new Pose(13.531, 34.956),

               new Pose(60.394, 84.254)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
         .build();


      follower.followPath(myPath.getPath(pathState));

      waitForStart();
      while (opModeIsActive()) {
         if (!follower.isBusy()) {
            if (pathState == 0 || pathState == 4 || pathState == 7 || pathState == 10) {


               while (!mechanisms.shoot(targetMotif)) {
                  mechanisms.update();
               }
               mechanisms.startIntake();
            } else if (pathState == 3 || pathState == 6 || pathState == 8) {
               mechanisms.slowIntake();
            }
            pathState++;
            if (pathState == 1 || pathState == 4 || pathState == 7) {
               follower.followPath(new PathChain(myPath.getPath(pathState)), false);
            } else if (pathState == 2 || pathState == 7 || pathState == 10) {
               follower.followPath(new PathChain(myPath.getPath(pathState)), intakeDriveSpeed, true);
            } else if (pathState == 3 || pathState == 4) {
               sleep(1500);
            } else {
               follower.followPath(myPath.getPath(pathState));
            }
         } else {
            if (pathState == 2 || pathState == 7 || pathState == 10) {
               mechanisms.startIntake();
            } else if (pathState == 3 || pathState == 7 || pathState == 10) {
               mechanisms.validateArtifacts();
            }

         }
         telemetry.addData("State: ", mechanisms.getState());
         mechanisms.update();
         follower.update();
      }
   }
}