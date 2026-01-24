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
import org.firstinspires.ftc.teamcode.Utils.ShootingSequence;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;


class Output {
   public int startIndex;
   public int endIndex;
   public double points;

   public Output(int startIndex, int endIndex, double points) {
      this.startIndex = startIndex;
      this.endIndex = endIndex;
      this.points = points;
   }
}


class Slot {
   int section;
   double position;
   int shootingIndex;

   public Slot(int section, double position, int shootingIndex) {
      this.section = section;
      this.position = position;
      this.shootingIndex = shootingIndex;
   }

   @Override
   public String toString() {
      return section + ":" + position;
   }
}

@Configurable
public class SpinIndexer {
   Servo leftSpinIndexer, rightSpinIndexer;
   SpindexerSection[] sections;
   private int currentPosition = 0;
   private boolean isCurrentIntake = false;


   public SpinIndexer(HardwareMap hardwareMap) {
      sections = new SpindexerSection[]{
         new SpindexerSection(0.670, new double[]{0.246, 1}), // 0th section
         new SpindexerSection(0.147, new double[]{0.508}),// 1st section
         new SpindexerSection(0.400, new double[]{0, 0.740})// 2nd section
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
      for (int i = 0; i < sections.length; i++) {
         if (sections[i].currentArtifact.matches(artifact)) {
            return i;
         }
      }
      return -1;
   }

   public double getServoPosition() {
      return rightSpinIndexer.getPosition();
   }

   private void setServoPosition(double position) {
      leftSpinIndexer.setPosition(position);
      rightSpinIndexer.setPosition(position);
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


   public ShootingSequence[] getBestShootingSequence(Artifact[] pattern) {
      double currentServoPosition = rightSpinIndexer.getPosition();
      Output[] start = new Output[10];
      List<Slot> slots = new ArrayList<>();

      // Collect all shooting positions
      for (int sectionIndex = 0; sectionIndex < sections.length; sectionIndex++) {
         double[] shootingPositions = sections[sectionIndex].getShootingPositions();

         for (int shootingIndex = 0; shootingIndex < shootingPositions.length; shootingIndex++) {
            slots.add(new Slot(sectionIndex, shootingPositions[shootingIndex], shootingIndex));
         }
      }

      // Sort by shooting position
      slots.sort(Comparator.comparingDouble(s -> s.position));

      int EndIndex = 0;
      // the pattern
      // state of spindexer


      int NumberofArtifacts = this.getNoOfArtifacts();


      if (NumberofArtifacts == 0) {
         return new ShootingSequence[0];
      }

      for (int startIndex = 0; EndIndex < slots.size() - 1; startIndex++) {
         EndIndex = startIndex + NumberofArtifacts - 1;
         double points = 0;
         int patternIndex = 0;

         for (int index = startIndex; index <= EndIndex; index++) {
            // iteration 1:
            if (getArtifact(slots.get(index).section).matches(pattern[patternIndex])) {
               points += 1;

            } else if (getArtifact(slots.get(index).section).matches(Artifact.ANY)) {
               // iteration 2
               points += 0.5;
            }
            patternIndex++;
         }
         start[startIndex] = new Output(startIndex, EndIndex, points);
      }

      int offset = slots.size();

      for (int startIndex = slots.size() - 1; startIndex >= 0; startIndex--) {

         int endIndex = startIndex - NumberofArtifacts + 1;
         if (endIndex < 0) {
            break;
         }
         double points = 0;
         int patternIndex = 0;

         for (int index = startIndex; index >= endIndex; index--) {
            if (getArtifact(slots.get(index).section).matches(pattern[patternIndex])) {
               points += 1;

            } else if (getArtifact(slots.get(index).section).matches(Artifact.ANY)) {
               // iteration 2
               points += 0.5;
            }
            patternIndex++;
         }

         start[startIndex + offset] = new Output(startIndex, endIndex, points);
      }


      double maxPoints = Double.MIN_VALUE;
      int bestindex = -1;
      for (int i = 0; i < start.length; i++) {


         if (start[i] != null && start[i].points > maxPoints) {
            maxPoints = start[i].points;
            bestindex = i;
         }
      }


      Output choosen = null;
      double bestDistance = Double.MAX_VALUE;

      for (int i = 0; i < start.length; i++) {
         if (start[i] == null) continue;

         double startPosition = slots.get(start[i].startIndex).position;

         double distance = Math.abs(startPosition - currentServoPosition);

         if (distance < bestDistance && maxPoints == start[i].points) {
            bestDistance = distance;
            choosen = start[i];
         }
      }


      int startIndex = choosen.startIndex;
      int endIndex = choosen.endIndex;

      if (startIndex < 0 || endIndex < 0
         || startIndex >= slots.size()
         || endIndex >= slots.size()) {
         return new ShootingSequence[0];
      }

      List<ShootingSequence> sequence = new ArrayList<>();

      if (startIndex > endIndex) {
         for (int i = startIndex; i >= endIndex; i--) {
            Slot s = slots.get(i);
            sequence.add(
               new ShootingSequence(s.section, s.shootingIndex)
            );
         }
      } else {
         for (int i = startIndex; i <= endIndex; i++) {
            Slot s = slots.get(i);
            sequence.add(
               new ShootingSequence(s.section, s.shootingIndex)
            );
         }
      }

      return sequence.toArray(new ShootingSequence[0]);


   }

   public int getNoOfArtifacts() {
      int NumberofArtifacts = 0;
      for (int i = 0; i < sections.length; i++) {
         if (sections[i].currentArtifact.matches(Artifact.ANY)) {
            NumberofArtifacts++;
         }
      }
      return NumberofArtifacts;
   }

   public String getSpinIndexerState() {
      String output = "";
      for (int i = 0; i < 3; i++) {
         output += (i + 1) + ": " + sections[i].currentArtifact;
      }
      return output;
   }
   public String getShootingSequenceString(Artifact[] pattern) {
      ShootingSequence[] sequence = this.getBestShootingSequence(pattern);

      if (sequence == null || sequence.length == 0) {
         return "EMPTY";
      }

      StringBuilder sb = new StringBuilder();

      for (int i = 0; i < sequence.length; i++) {
         ShootingSequence s = sequence[i];
         sb.append("[")
            .append(i)
            .append(": sec=")
            .append(s.section)
            .append(", idx=")
            .append(s.shootingIndex)
            .append("] ");

      }

      return sb.toString();
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
