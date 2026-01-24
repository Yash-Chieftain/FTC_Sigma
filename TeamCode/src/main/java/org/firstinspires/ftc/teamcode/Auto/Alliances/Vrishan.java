package org.firstinspires.ftc.teamcode.Auto.Alliances;

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
public class Vrishan extends LinearOpMode {
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
               new Pose(35.300, 135.978),

               new Pose(56.006, 84.404)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

         // Path2
         .addPath(
            new BezierLine(
               new Pose(56.006, 84.404),

               new Pose(45.729, 59.766)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))


         // Path3
         .addPath(
            new BezierLine(
               new Pose(45.729, 59.766),

               new Pose(20.418, 60.170)
            )
         ).setTangentHeadingInterpolation()
         // Path4
         .addPath(
            new BezierLine(
               new Pose(20.418, 60.170),

               new Pose(20.340, 69.446)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
         // Path5
         .addPath(
            new BezierLine(
               new Pose(20.418, 60.170),

               new Pose(20.340, 69.446)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

         // Path6
         .addPath(
            new BezierCurve(
               new Pose(11.713, 70.435),
               new Pose(38.733, 57.600),
               new Pose(55.724, 83.696)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

         // Path7
         .addPath(
            new BezierLine(
               new Pose(55.724, 83.696),

               new Pose(21.058, 84.357)
            )
         ).setTangentHeadingInterpolation()

         //Path8
         .addPath(
            new BezierLine(
               new Pose(21.058, 84.357),

               new Pose(55.646, 83.323)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
         //Path 9
         .addPath(
            new BezierLine(
               new Pose(55.646, 83.323),

               new Pose(17.471, 83.671)
            )
         ).setTangentHeadingInterpolation()
         .build();

      follower.followPath(myPath.getPath(pathState));
      mechanisms.setTurretTicks(-283);
      waitForStart();
      while (opModeIsActive()) {
         if (!follower.isBusy()) {
            if (pathState == 0 || pathState == 5 || pathState == 7) {
               while (mechanisms.getTurnValue() != 0) {
                  mechanisms.update();
                  mechanisms.startShooter();
               }
               while (!mechanisms.shoot(targetMotif)) {
                  mechanisms.update();
               }
               if (pathState == 0 || pathState == 5){mechanisms.startIntake();}
            }
            pathState++;
            if (pathState == 1 || pathState == 6 || pathState == 8) {
               follower.followPath(new PathChain(myPath.getPath(pathState)), false);
            } else if (pathState == 2 || pathState == 6 || pathState == 8) {
               follower.followPath(new PathChain(myPath.getPath(pathState)), intakeDriveSpeed, true);
            } else if (pathState == 4 || pathState == 3){
               sleep(2000);
            } else if (pathState == 2 || pathState == 6 || pathState == 5) {
               follower.followPath(new PathChain(PedroUtils.getPath(follower.getPose(), myPath.getPath(pathState).endPose())));
            } else {
               follower.followPath(myPath.getPath(pathState));
            }
         } else {
            if (pathState == 0) {
               mechanisms.readyToShoot(targetMotif);
               mechanisms.rampUpShooter();
            }
            if (pathState == 2 || pathState == 6 || pathState == 8) {
               mechanisms.startIntake();
               if (mechanisms.getNoOfArtifacts() >= 3) {
                  follower.breakFollowing();
               }
            }


            if (pathState == 4 || pathState == 6) {
               if(follower.getCurrentTValue() < 0.4){
                  mechanisms.readyToShoot(targetMotif);
               }else if (mechanisms.getNoOfArtifacts() < 3) {
                  mechanisms.startIntake();
               }else{
                  mechanisms.reverseIntake();
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
