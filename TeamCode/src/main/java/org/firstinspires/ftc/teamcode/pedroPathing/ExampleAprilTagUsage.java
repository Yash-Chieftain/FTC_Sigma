package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class ExampleAprilTagUsage extends OpMode {
   private final Pose TARGET_LOCATION = new Pose(); //Put the target location here
   private Limelight3A camera; //any camera here
   private Follower follower;
   private boolean following = false;

   @Override
   public void init() {
      camera = hardwareMap.get(Limelight3A.class, "limelight");
      camera.pipelineSwitch(1);
      follower = Constants.createFollower(hardwareMap);
   }

   @Override
   public void start() {
      camera.start();
   }

   @Override
   public void loop() {
      follower.update();
      telemetry.addData("Pose: ", getRobotPoseFromCamera().toString());
      telemetry.update();
      if (following && !follower.isBusy()) following = false;
   }

   private Pose getRobotPoseFromCamera() {
      LLResult result = camera.getLatestResult();
      if (result.isValid()) {
         Pose3D botpose = result.getBotpose();
         return new Pose(botpose.getPosition().x, botpose.getPosition().y, botpose.getOrientation().getYaw(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
      }
      return new Pose();
   }
}