package org.firstinspires.ftc.teamcode.Mechanisums;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Configurable
public class Turret {

    // ================= CONFIG =================

    public static double MAX_DEGREES = 300;
    public static double TICKS_PER_REV = 537.6;
    public static double GEAR_RATIO = 1.0;
    public static double MOTOR_POWER = 0.3;

    // PD gains
    public static double kp = 0.017;
    public static double kd = 0.025;   // 👈 added
    public static double maxPower = 0.5;
    public static double turretInitPower = 0.3;


    private DcMotorEx turretMotor;
    private DigitalChannel magneticSensor;
    private double targetDegrees = 0;
    // PD state
    private double lastError = 0;
    double minticklimit = Double.NEGATIVE_INFINITY;
    double maxtickslimit = Double.POSITIVE_INFINITY;


    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setPower(0);
        magneticSensor = hardwareMap.get(DigitalChannel.class, "magneticSensor");
        magneticSensor.setMode(DigitalChannel.Mode.INPUT);
//        while (!magneticSensor.getState()) {
//            turretMotor.setPower(turretInitPower);
//        }
//        maxtickslimit = turretMotor.getCurrentPosition();
//        while (!magneticSensor.getState()) {
//            turretMotor.setPower(-turretInitPower);
//        }
//        minticklimit = turretMotor.getCurrentPosition();
    }

    /**
     * Call every loop
     */
    public void update() {
        int targetTicks = degreesToTicks(targetDegrees);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(MOTOR_POWER);
    }

    public void setTargetDegrees(double deg) {
        targetDegrees = wrapDegrees(deg);
    }

    public int degreesToTicks(double deg) {
        return (int) ((deg / 360.0) * TICKS_PER_REV * GEAR_RATIO);
    }

    public double getCurrentDegrees() {
        int ticks = turretMotor.getCurrentPosition();
        return (ticks / (TICKS_PER_REV * GEAR_RATIO)) * 360.0;
    }

    public double getTicks() {
        return turretMotor.getCurrentPosition();
    }

    /**
     * PD control using Limelight tx
     */
    public void alignLimeLight(double tx) {
        double error = tx;
        double derivative = error - lastError;

        double power = (kp * error) + (kd * derivative);

        power = (Math.max(-maxPower, Math.min(maxPower, power)));

        if (power < 0) {
            if (turretMotor.getCurrentPosition() < minticklimit) {
                return;
            }
        } else if (power > 0) {
            if (turretMotor.getCurrentPosition() > maxtickslimit) {
                return;
            }

        }

        turretMotor.setPower(-power);

        lastError = error;
    }

    public double wrapDegrees(double deg) {
        deg %= MAX_DEGREES;
        if (deg < 0) deg += MAX_DEGREES;
        return deg;
    }

    public void stop() {
        turretMotor.setPower(0);
        lastError = 0;
    }

}
