//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import android.telephony.AccessNetworkConstants;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Mechanisums.Intake;
//import org.firstinspires.ftc.teamcode.Mechanisums.Shooter;
//import org.firstinspires.ftc.teamcode.Mechanisums.SpinIndexer;
//import org.firstinspires.ftc.teamcode.Mechanisums.Transfer;
//import org.firstinspires.ftc.teamcode.Utils.Artifact;
//
//@TeleOp
//public class MyTeleOp extends LinearOpMode {
//
//   Intake intake;
//   Shooter shooter;
//   Transfer transfer;
//   Artifact[] shootPattern = new Artifact[]{
//      Artifact.PURPLE,
//      Artifact.PURPLE,
//      Artifact.GREEN
//   };
//
//   @Override
//   public void runOpMode() throws InterruptedException {
//
//      intake = new Intake(hardwareMap);
//      shooter = new Shooter(hardwareMap);
//      transfer = new Transfer(hardwareMap);
//      transfer.resetAll();
//
//      waitForStart();
//      while (opModeIsActive()) {
//
//         if (gamepad1.left_bumper) {
//            intake.startIntake();
//            transfer.intake();
//         } else {
//            intake.slowIntake();
//         }
//
//         if (gamepad1.xWasPressed()) {
//            shootPattern = new Artifact[]{
//               Artifact.PURPLE,
//               Artifact.PURPLE,
//               Artifact.GREEN
//            };
//         } else if (gamepad1.yWasPressed()) {
//            shootPattern = new Artifact[]{
//               Artifact.PURPLE,
//               Artifact.GREEN,
//               Artifact.PURPLE
//            };
//         } else if (gamepad1.bWasPressed()) {
//            shootPattern = new Artifact[]{
//               Artifact.GREEN,
//               Artifact.PURPLE,
//               Artifact.PURPLE
//            };
//         }
//
//
//         if (gamepad1.aWasPressed()) {
//            Artifact lastArtifact;
//            boolean skipped = false;
//            for (int i = 0; i < shootPattern.length; i++) {
//               if (transfer.getNoOfArtifacts() > 0) {
//                  lastArtifact = transfer.shoot(shootPattern[i]);
//                  skipped = lastArtifact == Artifact.EMPTY;
//                  if(skipped || i >= 2 ){
//                     transfer.shootAll();
//                     break;
//                  }
//                  if ((lastArtifact != shootPattern[(i+1)]) && transfer.findArtifact(shootPattern[(i+1)])) {
//                     sleep(5000);
//                  }
//               }
//            }
//            if (transfer.getNoOfArtifacts() > 0) {
//               transfer.shootAll();
//            }
//            sleep(1000);
//            transfer.resetAll();
//         }
//
//         if (gamepad1.right_bumper) {
//            shooter.startShooter();
//         } else {
//            shooter.stopShooter();
//         }
//
//
//         telemetry.addData("Shoot Pattern:", getPattern());
//         telemetry.addData("State: ", transfer.getState());
//         telemetry.addData("Detected: ", transfer.getDetectedArtifacts());
//         telemetry.update();
//      }
//   }
//
//   String getPattern() {
//      String pattern = "";
//      for (int i = 0; i < shootPattern.length; i++) {
//         if (Artifact.GREEN == shootPattern[i]) {
//            pattern += (i + 1) + ": ," + "G";
//
//         }
//         if (Artifact.PURPLE == shootPattern[i]) {
//            pattern += (i + 1) + ": ," + "P";
//
//         }
//      }
//      return pattern;
//   }
//
//}
