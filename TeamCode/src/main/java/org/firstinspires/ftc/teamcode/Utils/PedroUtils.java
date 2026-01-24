package org.firstinspires.ftc.teamcode.Utils;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class PedroUtils {
   public static Pose[] pointsToPoses(double[][] points, double headingRadians) {
      Pose[] poses = new Pose[points.length];

      for (int i = 0; i < points.length; i++) {
         poses[i] = new Pose(
            points[i][0],
            points[i][1],
            headingRadians
         );
      }

      return poses;
   }
   public static Path getPath(Pose startPose, Pose endPose) {
      Path newPath = new Path(new BezierLine(startPose, endPose));
      newPath.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
      return newPath;
   }

   public static Path getPath(Pose startPose, Pose controlPoint, Pose endPose) {
      Path newPath = new Path(new BezierCurve(startPose, controlPoint, endPose));
      newPath.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
      return newPath;
   }
}
