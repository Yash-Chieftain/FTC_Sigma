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
import org.firstinspires.ftc.teamcode.Mechanisums.Shooter;
import org.firstinspires.ftc.teamcode.Utils.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class BFAR extends LinearOpMode {
   public static double intakeDriveSpeed= 0.35;
   Follower follower;
   int pathState = 0;
   long elapsed1;
   long elapsed2;
   long elapsed3;
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
            new BezierCurve(
               new Pose(55.606, 7.212),
               new Pose(67.425, 42.176),
               new Pose(11.819, 35.655)
            )
         )
         .addPath(
            new BezierLine(
               new Pose(11.819, 35.655),
               new Pose(57.241, 13.282)
            )
         )
         .addPath(
            new BezierCurve(
               new Pose(57.241, 13.282),
               new Pose(75.412, 69.337),
               new Pose(6.937, 59.477)
            )
         )
         .addPath(
            new BezierCurve(
               new Pose(6.937, 59.477),
               new Pose(35.345, 59.137),
               new Pose(58.692, 85.618)
            )
         )
         .addPath(
            new BezierLine(
               new Pose(58.692, 85.618),
               new Pose(17.505, 83.977)
            )
         )
         .addPath(
            new BezierLine(
               new Pose(17.505, 83.977),
               new Pose(58.315, 85.529)
            )
         )
         .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
         .build();


      follower.followPath(myPath.getPath(pathState));

      waitForStart();
      while (opModeIsActive()) {
         while (!mechanisms.shoot(targetMotif)) {
            mechanisms.update();
         }
         mechanisms.startIntake();
         if (!follower.isBusy()) {
            if (pathState == 1 || pathState == 3 || pathState == 5 ) {
               while (!mechanisms.shoot(targetMotif)) {
                  mechanisms.update();
               }
               mechanisms.startIntake();

            }
            pathState++;
         }if (pathState == 2 || pathState == 5) {
         } else if (pathState == 0) {
            long pathStartTime = System.currentTimeMillis();
            if(elapsed1== System.currentTimeMillis() - pathStartTime ){
               follower.followPath(new PathChain(myPath.getPath(pathState)), intakeDriveSpeed, true);
            } else if (pathState == 2 || pathState == 4) {
               sleep(500);
            }else if(pathState == 2){
               pathStartTime = System.currentTimeMillis();
               if (elapsed2 == System.currentTimeMillis() - pathStartTime){
                  follower.followPath(new PathChain(myPath.getPath(pathState)), intakeDriveSpeed, true);
               }
            }else if (pathState == 4){
               pathStartTime = System.currentTimeMillis();
               if(elapsed3 == System.currentTimeMillis() - pathStartTime){
                  follower.followPath(new PathChain(myPath.getPath(pathState)), intakeDriveSpeed, true);
               }
            }
            else {
               follower.followPath(myPath.getPath(pathState));
            }
         }
         if (pathState == 0 ){
            mechanisms.startLongshooter();
            mechanisms.startIntake();
         } else if (pathState == 3 || pathState == 6) {
            mechanisms.validateArtifacts();
         }

      }
      telemetry.addData("State: ", mechanisms.getState());
      mechanisms.update();
      follower.update();
   }
}