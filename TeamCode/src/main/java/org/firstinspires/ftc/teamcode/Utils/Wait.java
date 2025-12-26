package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Wait {
   public static void mySleep(long milliseconds){
      ElapsedTime timer = new ElapsedTime();
      timer.reset();
      while (timer.milliseconds() < milliseconds);
   }
}