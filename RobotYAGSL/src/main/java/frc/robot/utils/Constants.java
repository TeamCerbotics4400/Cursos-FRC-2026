package frc.robot.utils;

public class Constants {
    public class DrivetrainConstants {

        public static double TRACKWIDTH = 0.52;
        public static double TRACKLENGTH = 0.52;
    
        public static double BUMPER_HEIGHT = 0.4;
        
    }

    public class IntakeConstants{
        public static int intakeID = 20;
    }

    public class ShooterConstants {
        public static int upRight = 30;
        public static int downRight = 31;

        public static int upLeft = 32;
        public static int downLeft = 33;

        public static double kP = 0.18302;
        public static double kI = 0;
        public static double kD = 0;

        public static double kV = 0.11934;
        public static double kS = 0.0015086;
        public static double kA = 13312;

        public static double efficiency = 1;

        public static double maxRotationalVelocity = 0;

        public static double gearRatio = 1;
        public static double MOI = 0.05;
    }
}
