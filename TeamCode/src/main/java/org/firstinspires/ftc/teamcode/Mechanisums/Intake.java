package org.firstinspires.ftc.teamcode.Mechanisums;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Intake {
   public static double fastPower = 0.7;
   public static double slowPower = 0.4;
   public static double reversePower = -0.3;
   public static double stopServo = 0.135;

   DcMotor motor;
   Servo stopIntake;

   public Intake(HardwareMap hardwareMap) {
      motor = hardwareMap.get(DcMotor.class, "intake");
      stopIntake = hardwareMap.get(Servo.class, "stopIntake");
      stopIntake.setPosition(1);
      slowIntake();
   }

   public void startIntake() {
      motor.setPower(fastPower);
   }

   public void slowIntake() {
      motor.setPower(slowPower);

   }

   public void stopIntake() {
      stopIntake.setPosition(0.882);
   }

   public void reverse(){
      motor.setPower(reversePower);
   }
   public void reset(){
      stopIntake.setPosition(1);
   }

}
