package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisms;
import org.firstinspires.ftc.teamcode.Utils.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class AutoRed extends LinearOpMode {
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
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(109.710, 135.770, Math.toRadians(0)));
        follower.setMaxPower(0.8);
        mechanisms = new Mechanisms(hardwareMap);
        mechanisms.setSpinIndexerState(new Artifact[]{
                Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE
        });


        PathChain myPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(109.710, 135.770), new Pose(89.000, 83.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(53))
                .addPath(
                        new BezierLine(new Pose(89.000, 83.500), new Pose(100.000, 83.077))
                )
                .setLinearHeadingInterpolation(Math.toRadians(53), Math.toRadians(0))

                .addPath(
                        new BezierLine(new Pose(100.000, 83.077), new Pose(128.000, 83.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .addPath(
                        new BezierLine(new Pose(128.000, 83.500), new Pose(89.000, 83.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(51))

                .addPath(
                        new BezierLine(new Pose(89.000, 83.500), new Pose(100.000, 60.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(51), Math.toRadians(0))

                .addPath(
                        new BezierLine(new Pose(100.000, 60.500), new Pose(128.000, 60.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .addPath(
                        new BezierLine(new Pose(128.000, 60.500), new Pose(89.000, 83.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(51))

                .addPath(
                        new BezierLine(new Pose(89.000, 83.500), new Pose(100.000, 35.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(51), Math.toRadians(0))

                .addPath(
                        new BezierLine(new Pose(100.000, 35.500), new Pose(129.000, 35.500))
                )
                .setTangentHeadingInterpolation()

                .addPath(
                        new BezierLine(new Pose(129.000, 35.500), new Pose(89.000, 83.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(51))
                .build();


        follower.followPath(myPath.getPath(pathState));

        waitForStart();
        while (opModeIsActive()) {
            if (!follower.isBusy()) {
                if (pathState == 0 || pathState == 3 || pathState == 6 || pathState == 9) {


                    while (!mechanisms.shoot(targetMotif)) {
                        mechanisms.update();
                    }
                    mechanisms.startIntake();
                } else if (pathState == 2 || pathState == 5 || pathState == 8) {
                    mechanisms.slowIntake();
                } else if (pathState == 3) {
                    sleep(700);
                }
                pathState++;
                if (pathState == 1 || pathState == 4 || pathState == 7) {
                    follower.followPath(new PathChain(myPath.getPath(pathState)), false);
                } else if (pathState == 2 || pathState == 5 || pathState == 8) {
                    follower.followPath(new PathChain(myPath.getPath(pathState)), intakeDriveSpeed, true);
                } else {
                    follower.followPath(myPath.getPath(pathState));
                }
            } else {
                if (pathState == 2 || pathState == 5 || pathState == 8) {
                    mechanisms.startIntake();
                }
                if (pathState == 3 || pathState == 6 || pathState == 9) {
                    mechanisms.validateArtifacts();
                }

            }
            telemetry.addData("State: ", mechanisms.getState());
            mechanisms.update();
            follower.update();
        }
    }
}

