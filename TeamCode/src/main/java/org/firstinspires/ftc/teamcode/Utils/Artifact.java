package org.firstinspires.ftc.teamcode.Utils;

public enum Artifact {
   GREEN,
   PURPLE,
   EMPTY,
   ANY;

   public boolean matches(Artifact other) {
      if (other == ANY) {
         return this == GREEN || this == PURPLE;
      }
      return this == other;
   }
}
