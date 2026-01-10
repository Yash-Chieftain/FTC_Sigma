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
public class AutoBlueFar extends LinearOpMode {
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
        follower.setStartingPose(new Pose(34.298, 135.777, Math.toRadians(180)));
        follower.setMaxPower(0.8);
        mechanisms = new Mechanisms(hardwareMap);
        mechanisms.setSpinIndexerState(new Artifact[]{
                Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE
        });


        PathChain myPath = follower
                .pathBuilder()
                // shoot 1
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 8.222),

                                new Pose(60.591, 23.040)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))

                // intake align 1
                .addPath(
                        new BezierLine(
                                new Pose(60.591, 23.040),

                                new Pose(40.874, 36.332)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))

                // intake 1
                .addPath(
                        new BezierLine(
                                new Pose(40.874, 36.332),

                                new Pose(14.511, 36.332)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                // shoot 2
                .addPath(
                        new BezierLine(
                                new Pose(14.511, 36.332),

                                new Pose(59.040, 15.065)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))

                // intake align 2
                .addPath(
                        new BezierLine(
                                new Pose(59.040, 15.065),

                                new Pose(59.483, 60.702)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))

                // intake 2
                .addPath(
                        new BezierLine(
                                new Pose(59.483, 60.702),

                                new Pose(18.720, 60.480)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                // shoot 3
                .addPath(
                        new BezierLine(
                                new Pose(18.720, 60.480),

                                new Pose(59.040, 15.065)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))

                .build();

        waitForStart();

        while (opModeIsActive()) {

            if (!follower.isBusy()) {

                // -------- ACTIONS BEFORE MOVING TO NEXT PATH --------
                if (pathState == 0 || pathState == 3 || pathState == 6) {
                    // Shoot paths
                    while (!mechanisms.shoot(targetMotif)) {
                        mechanisms.update();
                    }
                }

                if (pathState == 2 || pathState == 5) {
                    // End of intake path
                    mechanisms.stopIntake();
                }

                // Move to next path
                pathState++;

                if (pathState >= 7) {
                    break; // all paths done
                }

                // -------- FOLLOW NEXT PATH --------
                if (pathState == 1 || pathState == 4) {
                    // Intake align paths
                    follower.followPath(
                            new PathChain(myPath.getPath(pathState)),
                            false
                    );

                } else if (pathState == 2 || pathState == 5) {
                    // Intake paths
                    mechanisms.startIntake();
                    follower.followPath(
                            new PathChain(myPath.getPath(pathState)),
                            intakeDriveSpeed,
                            true
                    );

                } else {
                    // Shoot paths
                    follower.followPath(myPath.getPath(pathState));
                }

            } else {
                // -------- ACTIONS WHILE MOVING --------
                if (pathState == 2 || pathState == 5) {
                    mechanisms.startIntake();
                }

                if (pathState == 3 || pathState == 6) {
                    mechanisms.validateArtifacts();
                }
            }
        }
            telemetry.addData("State: ", mechanisms.getState());
            mechanisms.update();
            follower.update();
        }
    }
