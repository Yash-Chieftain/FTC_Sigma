package org.firstinspires.ftc.teamcode.Mechanisums;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Configurable
public class Turret {
    public static double MAX_DEGREES = 300;
    public static double TICKS_PER_REV = 537.6;
    public static double GEAR_RATIO = 1.0;
    public static double MOTOR_POWER = 0.3;

    public static double kp = 0.017;
    public static double kd = 0.025;
    public static double maxPower = 0.5;
    public static double turretInitPower = 0.3;
    public static double turretOffset = 2;


    private DcMotorEx turretMotor;
    private DigitalChannel magneticSensor;
    private double targetDegrees = 0;
    // PD state
    private double lastError = 0;
    double minticklimit = -500;
    double maxtickslimit = 500;


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
//        maxtickslimit = tu  rretMotor.getCurrentPosition();
//        while (!magneticSensor.getState()) {
//            turretMotor.setPower(-turretInitPower);
//        }
//        minticklimit = turretMotor.getCurrentPosition();
    }


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


    public void setTurretTicks(int ticks){
        turretMotor.setTargetPosition(ticks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(maxPower);
    }

    public void alignLimeLight(double tx) {
        double error = tx + turretOffset;
        double derivative = error - lastError;

        double power = (kp * error) + (kd * derivative);

        power = (Math.max(-maxPower, Math.min(maxPower, power)));
        if(!turretMotor.isBusy()){
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (power > 0) {
            if (turretMotor.getCurrentPosition() < minticklimit) {
                turretMotor.setPower(0);
                return;
            }
        } else if (power < 0) {
            if (turretMotor.getCurrentPosition() > maxtickslimit) {
                turretMotor.setPower(0);
                return;
            }

        }

        turretMotor.setPower(-power);
        lastError = error;
    }

    public void setPower(double power){
        turretMotor.setPower(power);
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
