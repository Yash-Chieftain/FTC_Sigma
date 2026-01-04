package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.DataInput;

public class Constants {
   public static FollowerConstants followerConstants = new FollowerConstants()
      .mass(10.7)
      .forwardZeroPowerAcceleration(-47.162)
      .lateralZeroPowerAcceleration(-61.38)
      .centripetalScaling(0.0005)
      .translationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0.01, 0.03))
      .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.06, 0.01))
      .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.000001, 0.6, 0.03));
   ;

   public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
   public static MecanumConstants driveConstants = new MecanumConstants()
      .maxPower(1)
      .rightFrontMotorName("frontRight")
      .rightRearMotorName("backRight")
      .leftRearMotorName("backLeft")
      .leftFrontMotorName("frontLeft")
      .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
      .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
      .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
      .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
      .xVelocity(58.6)
      .yVelocity(45.5);
   public static PinpointConstants localizerConstants = new PinpointConstants()
      .forwardPodY(-3.3464)
      .strafePodX(1.4763)
      .distanceUnit(DistanceUnit.INCH)
      .hardwareMapName("pinpoint")
      .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
      .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
      .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

   public static Follower createFollower(HardwareMap hardwareMap) {
      return new FollowerBuilder(followerConstants, hardwareMap)
         .pathConstraints(pathConstraints)
         .mecanumDrivetrain(driveConstants)
         .pinpointLocalizer(localizerConstants)
         .build();
   }
}
