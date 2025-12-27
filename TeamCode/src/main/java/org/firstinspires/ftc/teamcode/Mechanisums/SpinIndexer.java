package org.firstinspires.ftc.teamcode.Mechanisums;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utils.Artifact;

@Configurable
public class SpinIndexer {
   Servo leftSpinIndexer, rightSpinIndexer;
   SpindexerSection[] sections;
   private int currentPosition = 0;
   private boolean isCurrentIntake = false;


   public SpinIndexer(HardwareMap hardwareMap) {
      sections = new SpindexerSection[]{
         new SpindexerSection(0.63, new double[]{0.234, 1}), // 0th section
         new SpindexerSection(0.137, new double[]{0.487}),// 1st section
         new SpindexerSection(0.371, new double[]{0, 0.737})// 2nd section
      };
      leftSpinIndexer = hardwareMap.get(Servo.class, "leftSpinIndexer");
      rightSpinIndexer = hardwareMap.get(Servo.class, "rightSpinIndexer");
      this.setPosition(0);
   }

   public int getCurrentPosition() {
      return currentPosition;
   }

   public boolean getIsCurrentIntake() {
      return isCurrentIntake;
   }


   public Artifact getArtifact(int index) {
      return sections[index].currentArtifact;
   }
   public int getArtifact(Artifact artifact) {
      for(int i = 0; i< sections.length; i++){
         if(sections[i].currentArtifact.matches(artifact)){
            return i;
         }
      }
      return -1;
   }

   private void setServoPosition(double position){
      leftSpinIndexer.setPosition(0.04 + position);
      rightSpinIndexer.setPosition(position);
   }

   public double getServoPosition(){
      return rightSpinIndexer.getPosition();
   }


   public void setPosition(int index) {
      setServoPosition(sections[index].getIntakePosition());
      currentPosition = index;
      isCurrentIntake = true;
   }

   public void setPosition(int index, int shootingPositionIndex) {
      setServoPosition(sections[index].getShootingPositions()[shootingPositionIndex]);
      currentPosition = index;
      isCurrentIntake = false;
   }

   public boolean setPosition(Artifact artifact, boolean isIntake) {
      double distance = Double.POSITIVE_INFINITY;
      int sectionIndex = 0, shootingIndex = -1;
      boolean isArtifactThere = false;


      for (int i = 0; i < sections.length; i++) {
         if (sections[i].currentArtifact.matches(artifact)) {
            isArtifactThere = true;
            if (isIntake) {
               double distanceError = Math.abs(sections[i].getIntakePosition() - getServoPosition());
               if (distanceError < distance) {
                  sectionIndex = i;
                  distance = distanceError;
               }
            } else {
               double[] shootingPositions = sections[i].getShootingPositions();
               for (int index = 0; index < shootingPositions.length; index++) {
                  double distanceError = Math.abs(shootingPositions[index] - getServoPosition());
                  if (distanceError < distance) {
                     distance = distanceError;
                     sectionIndex = i;
                     shootingIndex = index;
                  }
               }
            }
         }
      }
      if (isArtifactThere) {
         if (isIntake) setPosition(sectionIndex);
         else setPosition(sectionIndex, shootingIndex);
      }
      return isArtifactThere;
   }

   public void setSectionArtifact(int index, Artifact artifact) {
      sections[index].currentArtifact = artifact;
   }

   public String getSpinIndexerState() {
      String output = "";
      for (int i = 0; i < 3; i++) {
         output += (i + 1) + ": " + sections[i].currentArtifact;
      }
      return output;
   }
}

class SpindexerSection {
   public Artifact currentArtifact;
   private double intakeServoPosition;
   private double[] shootingServoPositions;

   public SpindexerSection(double intakeServoPosition, double[] shootingPositions) {
      this.intakeServoPosition = intakeServoPosition;
      this.shootingServoPositions = shootingPositions;
      currentArtifact = Artifact.EMPTY;
   }


   public double getIntakePosition() {
      return intakeServoPosition;
   }

   public double[] getShootingPositions() {
      return shootingServoPositions;
   }
}
