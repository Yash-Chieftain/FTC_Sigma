package org.firstinspires.ftc.teamcode.Utils;

public class Motif {
   public static Artifact[] getMotif(int id) {
      if (id == 21) {
         return new Artifact[]{
            Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE
         };

      }
      if (id == 22) {
         return new Artifact[]{
            Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE
         };

      }
      if (id == 23) {
         return new Artifact[]{
            Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN
         };

      }
      return new Artifact[]{
         Artifact.PURPLE, Artifact.PURPLE, Artifact.PURPLE
      };
   }
}
