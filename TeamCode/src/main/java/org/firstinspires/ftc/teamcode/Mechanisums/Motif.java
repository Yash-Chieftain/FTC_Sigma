package org.firstinspires.ftc.teamcode.Mechanisums;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Artifact;

import java.util.List;

@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
@Disabled
public class Motif extends LinearOpMode {
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        Artifact[][] pattern = new Artifact[][]{
                {Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN},
                {Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE},
                {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE},
        };
        Artifact[] Motif;
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
        for (LLResultTypes.BarcodeResult br : barcodeResults) {
            System.out.println("Barcode ID: " + br.getData());
            if (br.getData().equals("21")) {
                Motif = pattern[2];

            } else if (br.getData().equals("23")) {
                Motif = pattern[0];
            }
            if (br.getData().equals("22")) {
                Motif = pattern[1];
            }
        }
    }
}



