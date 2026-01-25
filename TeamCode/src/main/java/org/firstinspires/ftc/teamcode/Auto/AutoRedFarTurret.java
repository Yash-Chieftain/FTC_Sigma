package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisms;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Utils.Artifact;
import org.firstinspires.ftc.teamcode.Utils.Motif;
import org.firstinspires.ftc.teamcode.Utils.PedroUtils;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class AutoRedFarTurret extends LinearOpMode {
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
        follower.setStartingPose(new Pose(87.000, 8.000, Math.toRadians(0)));
        follower.setMaxPower(0.95);
        mechanisms.setTurretTicks(550);
        mechanisms = new Mechanisms(hardwareMap);
        mechanisms.setSpinIndexerState(new Artifact[]{
                Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE
        });

        PathChain myPath = follower
                .pathBuilder()
                //Path 0
                .addPath(
                        new BezierCurve(
                                new Pose(87.000, 8.000),
                                new Pose(87, 13)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                // Path1
                .addPath(
                        new BezierCurve(
                                new Pose(87.082, 13),
                                new Pose(78.647, 36.349),
                                new Pose(103.997, 35.354)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )

                // Path2
                .addPath(
                        new BezierLine(
                                new Pose(103.997, 35.354),
                                new Pose(131.049, 35.314)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )

                // Path3
                .addPath(
                        new BezierLine(
                                new Pose(131.049, 35.314),
                                new Pose(86.029, 15.515)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )

                // Path4
                .addPath(
                        new BezierCurve(
                                new Pose(86.029, 15.515),
                                new Pose(75.442, 59.463),
                                new Pose(104.645, 59.648)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )

                // Path5
                .addPath(
                        new BezierLine(
                                new Pose(104.645, 59.648),
                                new Pose(131.218, 59.654)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )

                // Path6
                .addPath(
                        new BezierLine(
                                new Pose(131.218, 59.654),
                                new Pose(85.099, 14.420)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )

                // Path7
                .addPath(
                        new BezierLine(
                                new Pose(85.099, 14.420),
                                new Pose(125.016, 67.475)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(90)
                )

                .build();
        mechanisms.pipelineSwitch(RobotConstants.Mechanisms.Vision.motifPipelineIndex);
        mechanisms.setTurretTicks(550);
        waitForStart();
        while (opModeIsActive()) {
            if (!follower.isBusy()) {
                if (pathState == 0) {
                    targetMotif = Motif.getMotif(mechanisms.getId());
                    mechanisms.pipelineSwitch(RobotConstants.Mechanisms.Vision.blueAllianceGoalPipelineIndex);
                }
                if (pathState == 0 || pathState == 3 || pathState == 6 || pathState == 9) {
                    while (mechanisms.getTurnValue() != 0) {
                        mechanisms.update();
                        mechanisms.startShooter();
                    }
                    while (!mechanisms.shoot(targetMotif)) {
                        telemetry.addData("State: ", mechanisms.getState());
                        telemetry.addData("targetMotif: ", targetMotif[0] + "," + targetMotif[1] + "," + targetMotif[2]);
                        telemetry.update();
                        mechanisms.update();
                    }
                    mechanisms.startIntake();
                }
                pathState++;
                if (pathState == 1 || pathState == 4 || pathState == 7) {
                    follower.followPath(new PathChain(myPath.getPath(pathState)), false);
                } else if (pathState == 2 || pathState == 5 || pathState == 8) {
                    follower.followPath(new PathChain(myPath.getPath(pathState)), intakeDriveSpeed, true);
                } else if (pathState == 3 || pathState == 6 || pathState == 9) {
                    follower.followPath(new PathChain(PedroUtils.getPath(follower.getPose(), myPath.getPath(pathState).endPose())));
                } else {
                    follower.followPath(myPath.getPath(pathState));
                }
            } else {
                if (pathState == 0) {
                    mechanisms.rampUpShooter();
                }
                if (pathState == 2 || pathState == 5 || pathState == 8) {
                    mechanisms.startIntake();
                    if (mechanisms.getNoOfArtifacts() >= 3) {
                        follower.breakFollowing();
                    }
                }

                if (pathState == 3 || pathState == 6 || pathState == 9) {
                    if (follower.getCurrentTValue() < 0.4 && !(mechanisms.getNoOfArtifacts() < 3)) {
                        mechanisms.startIntake();
                    }
                    mechanisms.rampUpShooter();
                }
            }
            telemetry.addData("State: ", mechanisms.getState());
            telemetry.addData("ID", mechanisms.getId());
            telemetry.addData("targetMotif: ", targetMotif[0] + "," + targetMotif[1] + "," + targetMotif[2]);
            telemetry.update();
            mechanisms.update();
            follower.update();
        }
    }
}
