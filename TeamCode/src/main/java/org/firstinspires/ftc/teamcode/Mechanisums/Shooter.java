package org.firstinspires.ftc.teamcode.Mechanisums;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Wait;

@Configurable
public class Shooter {
    public static double velocity = 1200;
    public static double kp = 250;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 19;
    public static long shootDelay = 100;
    public static double shootPosition = 0.700;
    public static double divideShootTime = 1;
    DcMotorEx leftMotor, rightMotor;
    Servo leftHood, rightHood;
    Servo bootKicker1, bootKicker2;

    public Shooter(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftshoot");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightshoot");
        leftHood = hardwareMap.get(Servo.class, "leftHood");
        rightHood = hardwareMap.get(Servo.class, "rightHood");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftHood.setDirection(Servo.Direction.REVERSE);
        rightHood.setPosition(0);
        leftHood.setPosition(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kp, ki, kd, kf)
        );
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kp, ki, kd, kf)
        );
        bootKicker1 = hardwareMap.get(Servo.class, "bootKicker1");
        bootKicker1.setDirection(Servo.Direction.REVERSE);
        bootKicker1.setPosition(0.01);
        bootKicker2 = hardwareMap.get(Servo.class, "bootKicker2");
        bootKicker2.setPosition(0.01);
    }

    public void startShooter() {
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kp, ki, kd, kf)
        );
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kp, ki, kd, kf)
        );

        leftMotor.setVelocity(velocity);
        rightMotor.setVelocity(velocity);
    }

    public void stopShooter() {
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public double getVelocity() {
        return leftMotor.getVelocity();
    }

    public double getTargetVelocity() {
        return velocity;
    }

    public boolean isVelocityReached() {
        return Math.abs(getVelocity() - velocity) < 30;
    }


    public void shoot() {
        if (this.isVelocityReached()) {
            bootKicker1.setPosition(shootPosition);
            bootKicker2.setPosition(shootPosition);
            Wait.mySleep(shootDelay);
            bootKicker1.setPosition(0.01);
            bootKicker2.setPosition(0.01);
            Wait.mySleep((long) (shootDelay / divideShootTime));
        }
    }

    public void setHoodPosition(double position) {
        leftHood.setPosition(position);
        rightHood.setPosition(position);
   }
}

