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
public class RVauto extends LinearOpMode {
   public static double intakeDriveSpeed = 0.285;

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
      follower.setStartingPose(new Pose(33.740, 136.832, Math.toRadians(180)));
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
               new Pose(33.939, 135.955),

               new Pose(52.147, 84.234)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)
         )

         // Path2
         .addPath(
            new BezierLine(
               new Pose(52.147, 84.234),

               new Pose(43.724, 84.234)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)
         )

         // Path3
         .addPath(
            new BezierLine(
               new Pose(43.724, 84.234),

               new Pose(19.055, 84.033)
            )
         ).setTangentHeadingInterpolation()

         // Path4
         .addPath(
            new BezierLine(
               new Pose(19.055, 84.033),

               new Pose(52.348, 84.033)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)
         )

         // Path5
         .addPath(
            new BezierLine(
               new Pose(52.348, 84.033),

               new Pose(47.935, 60.499)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

         // Path6
         .addPath(
            new BezierLine(
               new Pose(47.935, 60.499),

               new Pose(17.384, 60.031)
            )
         ).setTangentHeadingInterpolation()



         // Path7
         .addPath(
            new BezierLine(
               new Pose(17.384, 60.031),

               new Pose(52.234, 83.596)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

         .build();
      follower.followPath(myPath.getPath(pathState));

      waitForStart();
      while (opModeIsActive()) {
         if (!follower.isBusy()) {
            if (pathState == 0 || pathState == 3 || pathState == 6) {
               while (!mechanisms.shoot(targetMotif)) {
                  mechanisms.update();
               }
               mechanisms.startIntake();

            } else if (pathState == 2 || pathState == 5 ) {
               mechanisms.slowIntake();
            }
            pathState++;
            if (pathState == 1 || pathState == 4 ) {
               follower.followPath(new PathChain(myPath.getPath(pathState)), false);
            } else if (pathState == 2 || pathState == 5 ) {
               follower.followPath(new PathChain(myPath.getPath(pathState)), intakeDriveSpeed, true);
            } else if (pathState == 2 || pathState == 5) {
               sleep(500);
            } else {
               follower.followPath(myPath.getPath(pathState));
            }
         } else {
            if (pathState == 2 || pathState == 5 || pathState == 10) {
               mechanisms.startIntake();
               mechanisms.startShooter();
            } else if (pathState == 2 || pathState == 6 ) {
               mechanisms.validateArtifacts();
            }

         }
         telemetry.addData("State: ", mechanisms.getState());
         mechanisms.update();
         follower.update();
      }
   }
}