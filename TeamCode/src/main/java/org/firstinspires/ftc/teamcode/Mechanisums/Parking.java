package org.firstinspires.ftc.teamcode.Mechanisums;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Parking {
    Servo leftPark, rightPark;
    public static double parkPosition = 0.85;
    public Parking(HardwareMap hardwareMap){
        leftPark = hardwareMap.get(Servo.class, "leftPark");
        rightPark = hardwareMap.get(Servo.class,"rightPark");
        rightPark.setDirection(Servo.Direction.REVERSE);
        leftPark.setPosition(0);
        rightPark.setPosition(0);
    }

    public void park(){
        leftPark.setPosition(parkPosition);
        rightPark.setPosition(parkPosition);

    }

    public void reset(){
        leftPark.setPosition(0);
        rightPark.setPosition(0);
    }
}
