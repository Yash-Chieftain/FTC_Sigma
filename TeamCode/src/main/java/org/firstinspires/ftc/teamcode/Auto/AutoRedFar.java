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
public class AutoRedFar extends LinearOpMode {
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
//               shoot 1
                .addPath(
                        new BezierLine(
                                new Pose(86.732, 8.640),

                                new Pose(83.695, 12.748)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68))
//                intake align 1
                .addPath(
                        new BezierLine(
                                new Pose(83.695, 12.748),

                                new Pose(102.233, 35.889)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
//                intake 1
                .addPath(
                        new BezierLine(
                                new Pose(102.233, 35.889),

                                new Pose(123.951, 35.889)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//              shoot 2
                .addPath(
                        new BezierLine(
                                new Pose(123.951, 35.889),

                                new Pose(85.691, 16.517)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
//                intake align 2
                .addPath(
                new BezierLine(
                        new Pose(85.691, 16.517),

                        new Pose(85.691, 59.085)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
//                inake 2
                .addPath(
                        new BezierLine(
                                new Pose(85.691, 59.085),

                                new Pose(125.342, 59.151)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                shoot 3
                .addPath(
                        new BezierLine(
                                new Pose(125.342, 59.151),

                                new Pose(85.912, 16.517)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();


        follower.followPath(myPath.getPath(pathState));

        waitForStart();
        while (opModeIsActive()) {
            if (!follower.isBusy()) {

                // -------- ACTIONS AT END OF CURRENT PATH --------
                if (pathState == 0 || pathState == 3 || pathState == 6) {
                    // Shoot paths
                    while (!mechanisms.shoot(targetMotif)) {
                        mechanisms.update();
                    }
                }

                if (pathState == 2 || pathState == 5) {
                    // Finished intake path
                    mechanisms.stopIntake();
                }

                // Advance state
                pathState++;

                // Stop once all paths are done
                if (pathState >= 7) {
                    break;
                }
                if (pathState == 1 || pathState == 4) {
                    // Intake align
                    follower.followPath(
                            new PathChain(myPath.getPath(pathState)),
                            false
                    );

                } else if (pathState == 2 || pathState == 5) {
                    // Intake
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


