package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.teamcode.Mechanisums.Mechanisms;
import org.firstinspires.ftc.teamcode.Mechanisums.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisums.SpinIndexer;
import org.firstinspires.ftc.teamcode.Mechanisums.Turret;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Utils.Artifact;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class Test extends LinearOpMode {
    Mechanisms mechanisms;
    Follower follower;
    public static double maxPower = 1;
    int spinIndexerIndex = 0;

    boolean isPark = true;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        mechanisms = new Mechanisms(hardwareMap);
        waitForStart();
        follower.startTeleopDrive(true);
        follower.update();
        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                maxPower = 0.5;
            } else if (gamepad1.left_trigger > 0.5) {
                maxPower = 0.2;
            } else if (gamepad1.right_trigger > 0.5) {
                maxPower = -1;
            } else {
                maxPower = 1;
            }

            if (gamepad2.left_bumper && gamepad2.bWasPressed()) {
                spinIndexerIndex += 1;
                spinIndexerIndex = spinIndexerIndex % 3;
                mechanisms.setSpinIndexerIntakePosition(spinIndexerIndex);
            } else if (gamepad2.bWasPressed()) {
                spinIndexerIndex += 1;
                spinIndexerIndex = spinIndexerIndex % 3;
                mechanisms.setSpinIndexerShootingPosition(spinIndexerIndex);
            }

            if(gamepad1.yWasPressed()){
                if(isPark){
                    isPark = false;
                    mechanisms.unPark();
                }else{
                    isPark = true;
                    mechanisms.park();
                }
            }


            if (gamepad2.left_bumper) {
                mechanisms.startIntake();
            } else if (gamepad2.left_trigger > 0.5) {
                mechanisms.reverseIntake();
            } else {
                mechanisms.stopIntake();
            }


            if (gamepad2.right_bumper) {
                mechanisms.startShooter();
            } else {
                mechanisms.stopShooter();
            }
            if (gamepad2.dpad_up){

            }

            if(gamepad2.right_bumper && gamepad2.aWasPressed()){
                mechanisms.shoot();
            }else if (gamepad2.a) {
                mechanisms.shoot(new Artifact[]{Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE});
            }

            follower.setMaxPower(maxPower);
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            telemetry.addData("Velocity", mechanisms.getShooterVelocity());
            telemetry.addData("State", mechanisms.getState());
            telemetry.addData("Color", mechanisms.getColorSensorValues());
            telemetry.addData("getTx", mechanisms.getTurnValue());
            telemetry.update();
            mechanisms.update();
        }
    }
}