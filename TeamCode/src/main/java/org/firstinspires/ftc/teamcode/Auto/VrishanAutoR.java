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
public class VrishanAutoR extends LinearOpMode {
   public static double intakeDriveSpeed = 0.28;
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
               new Pose(110.960, 136.044),

               new Pose(88.700, 81.899)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))


         // Path2
         .addPath(
            new BezierLine(
               new Pose(88.700, 81.899),

               new Pose(101.605, 83.469)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)
         )

         // Path3
         .addPath(
            new BezierLine(
               new Pose(101.605, 83.469),

               new Pose(126.975, 84.014)
            )
         ).setTangentHeadingInterpolation()

         // Path4
         .addPath(
            new BezierLine(
               new Pose(126.975, 84.014),

               new Pose(89.171, 82.051)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)
         )

         // Path5
         .addPath(
            new BezierLine(
               new Pose(89.171, 82.051),

               new Pose(96.910, 59.364)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))


         // Path6
         .addPath(
            new BezierLine(
               new Pose(96.910, 59.364),

               new Pose(126.193, 59.528)
            )
         ).setTangentHeadingInterpolation()


         // Path7
         .addPath(
            new BezierLine(
               new Pose(126.193, 59.528),

               new Pose(88.653, 82.190)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
         .build();


      follower.followPath(myPath.getPath(pathState));

      waitForStart();
      while (opModeIsActive()) {
         if (!follower.isBusy()) {
            if (pathState == 0 || pathState == 3 || pathState == 6 ) {
               while (!mechanisms.shoot(targetMotif)) {
                  mechanisms.update();
               }
               mechanisms.startIntake();

            } else if (pathState == 2 || pathState == 5) {
               mechanisms.slowIntake();
            }
            pathState++;
            if (pathState == 1 || pathState == 4 ) {
               follower.followPath(new PathChain(myPath.getPath(pathState)), false);
            } else if (pathState == 2 || pathState == 5) {
               follower.followPath(new PathChain(myPath.getPath(pathState)), intakeDriveSpeed, true);
            } else if (pathState == 2 || pathState == 4) {
               sleep(500);
            } else {
               follower.followPath(myPath.getPath(pathState));
            }
         } else {
            if (pathState == 2 || pathState == 5) {
               mechanisms.startIntake();
               mechanisms.startShooter();
            } else if (pathState == 3 || pathState == 6) {
               mechanisms.validateArtifacts();
            }

         }
         telemetry.addData("State: ", mechanisms.getState());
         mechanisms.update();
         follower.update();
      }
   }
}