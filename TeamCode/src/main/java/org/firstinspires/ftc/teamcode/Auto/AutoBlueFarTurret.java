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
public class AutoBlueFarTurret extends LinearOpMode {
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
        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(180)));
        follower.setMaxPower(0.95);
        mechanisms = new Mechanisms(hardwareMap);
        mechanisms.setSpinIndexerState(new Artifact[]{
                Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE
        });

        PathChain myPath = follower
                .pathBuilder()
                //Path 0
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.000),
                                new Pose(56, 13)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                // Path1
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 13.000),
                                new Pose(63.833, 37.396),
                                new Pose(42.000, 35.500)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )

                // Path2
                .addPath(
                        new BezierLine(
                                new Pose(42.000, 35.500),
                                new Pose(15.000, 35.500)
                        )
                )
                .setTangentHeadingInterpolation()

                // Path3
                .addPath(
                        new BezierLine(
                                new Pose(15.000, 35.500),
                                new Pose(56.000, 13.000)
                        )
                )
                .setConstantHeadingInterpolation(
                        Math.toRadians(180)
                )

                // Path4
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 13.000),
                                new Pose(72.267, 61.185),
                                new Pose(42.000, 59.500)
                        )
                )
                .setConstantHeadingInterpolation(
                        Math.toRadians(180)
                )

                // Path5
                .addPath(
                        new BezierLine(
                                new Pose(42.000, 59.500),
                                new Pose(15.000, 59.500)
                        )
                )
                .setTangentHeadingInterpolation()

                // Path6
                .addPath(
                        new BezierLine(
                                new Pose(15.000, 59.500),
                                new Pose(56.000, 13.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )

                // Path7
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 13.000),
                                new Pose(50.000, 13.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        mechanisms.pipelineSwitch(RobotConstants.Mechanisms.Vision.motifPipelineIndex);
        mechanisms.setTurretTicks(-550);
        mechanisms.setTurretOffset(-2);
        waitForStart();
        while (opModeIsActive()) {
            if (!follower.isBusy()) {
                if (pathState == 0 || pathState == 3 || pathState == 6 || pathState == 9) {
                    targetMotif = Motif.getMotif(mechanisms.getId());
                    mechanisms.pipelineSwitch(RobotConstants.Mechanisms.Vision.blueAllianceGoalPipelineIndex);
                    mechanisms.setTurretTicks(-300);
                    while(mechanisms.isTurretBusy()){
                        mechanisms.setTurretTicks(-300);
                    }
                    while (mechanisms.isTurretAligned()) {
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
                    mechanisms.readyToShoot(targetMotif);
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
