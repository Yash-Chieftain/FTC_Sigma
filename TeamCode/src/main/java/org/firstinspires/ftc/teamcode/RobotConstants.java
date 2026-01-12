package org.firstinspires.ftc.teamcode;
public class RobotConstants {
    public static class Auto{
        public static double intakeDriveSpeed = 0.25;
    }

    public static class TeleOp{
        public static double maxPower = 1;
    }

    public static  class Mechanisms{
        public static class Intake{
            public static double fastPower = 0.7;
            public static double slowPower = 0.4;
            public static double reversePower = -0.3;
        }
        public static class Shooter{
            public static double shooterPower = 0.85;
            public static void setShooterPower(double shooterPower){
                Shooter.shooterPower=shooterPower;
            }
        }
        public static class SpinIndexer{
            public static long movementSleep = 200;
            public static long actionWait = 200;
        }
        public static class Vision{
            public static double kP = 0.04;
            public static double kD = 0.004;
            public static double maxPower = 0.45;
            public static double dist_tolerance = 2;
            private static double limelightAngleMounted = 12;
            private static double limelightLensHeight = 11.5;
            private static double goalHeightInches = 29.5;
        }
        public static class Turret{
            public static double MAX_DEGREES = 300;
            public static double TICKS_PER_REV = 537.6;
            public static double GEAR_RATIO = 1.0;
            public static double MOTOR_POWER = 0.3;
            public static double kp = 0.017;
            public static double kd = 0.025;
            public static double maxPower = 0.5;
            public static double turretInitPower = 0.3;
        }

    }


}
