package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    public static class FieldConstants {
        public static final double FIELD_LENGTH = 16.51;
        public static final double FIELD_WIDTH = 8.04;

        public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.61, FIELD_WIDTH / 2.0);
        public static final Translation2d RED_HUB_CENTER =
                new Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2.0);
        public static final double HUB_ENTRY_HEIGHT_METERS = 1.83;
    }

    public static class DrivetrainConstants {
        public static double TRACKWIDTH = 0.52;
        public static double TRACKLENGTH = 0.52;

        // Dimensiones exteriores con bumpers para colisiones de chasis (coincide con PathPlanner settings.json)
        public static final double BUMPER_WIDTH = 0.90;
        public static final double BUMPER_LENGTH = 0.90;
        public static double BUMPER_HEIGHT = 0.4;
        public static final double ROBOT_TOTAL_HEIGHT = 0.50; // Menor a 0.565m -> pasa bajo el Trench

        // Controlador dedicado para apuntado al Hub / Shoot-on-the-Move
        public static final double AIM_KP = 6.0;
        public static final double AIM_KI = 0.0;
        public static final double AIM_KD = 0.2;
        public static final double MAX_AIM_ANGULAR_VEL_RAD_PER_SEC = Math.toRadians(540.0);
        public static final double MAX_AIM_ANGULAR_ACCEL_RAD_PER_SEC_SQ = Math.toRadians(900.0);
        public static final double AIM_TOLERANCE_DEG = 2.5;
    }

    public static class IntakeConstants {
        public static int intakeID = 20;
        public static final int MAX_FUEL_CAPACITY = 60;
        public static final double INTAKE_VOLTAGE = 12.0;
    }

    public static class FeederConstants {
        public static final int feederID = 25;
        public static final double FEED_VOLTAGE = 8.0;

        // Cadencia de disparo en simulación cuando el Feeder alimenta hacia el Shooter
        public static final double SHOTS_PER_SECOND = 6.0;
        public static final double SHOT_PERIOD_SEC = 1.0 / SHOTS_PER_SECOND;

        public static final double GEAR_RATIO = 3.0;
        public static final double MOI = 0.002;
    }

    public static class ShooterConstants {
        public static int upRight = 30;
        public static int downRight = 31;

        public static int upLeft = 32;
        public static int downLeft = 33;

        public static int hoodID = 34;

        // Ganancias de velocidad en Phoenix 6 (unidades en Volts por RPS / RPS^2)
        public static double kP = 0.18302;
        public static double kI = 0.0;
        public static double kD = 0.0;

        public static double kV = 0.11934;
        public static double kS = 0.0015086;
        public static double kA = 0.13312;

        public static double efficiency = 1.0;
        public static double maxRotationalVelocity = 6000.0;

        public static double gearRatio = 1.0;
        public static double MOI = 0.05;

        public static final double VELOCITY_TOLERANCE_RPM = 75.0;

        // Parámetros físicos de lanzamiento para conversión RPM <-> velocidad de salida
        public static final double WHEEL_RADIUS_METERS = 0.0508; // Rueda de 4 pulgadas (radio 2 in)
        public static final double BALL_TRANSFER_EFFICIENCY = 0.55;
        public static final double LAUNCH_HEIGHT_METERS = 0.55; // Mayor a BUMPER_HEIGHT (0.40 m)

        // Parámetros del Hood variable (0° = horizontal, 90° = vertical hacia arriba)
        public static final double MIN_HOOD_ANGLE_DEG = 45.0;
        public static final double MAX_HOOD_ANGLE_DEG = 80.0;
        public static final double DEFAULT_HOOD_ANGLE_DEG = 65.0;
        public static final double HOOD_TOLERANCE_DEG = 1.5;
        public static final double HOOD_GEAR_RATIO = 50.0;
        public static final double HOOD_MOI = 0.02;

        // Ganancias del Hood
        public static final double HOOD_KP = 24.0;
        public static final double HOOD_KD = 0.5;

        /** Convierte RPM del flywheel en velocidad lineal de salida de la pelota (m/s). */
        public static double rpmToLaunchVelocityMps(double rpm) {
            double omegaRadPerSec = rpm * (2.0 * Math.PI / 60.0);
            return omegaRadPerSec * WHEEL_RADIUS_METERS * BALL_TRANSFER_EFFICIENCY;
        }

        /** Convierte velocidad lineal deseada de salida (m/s) en RPM del flywheel. */
        public static double launchVelocityMpsToRPM(double velocityMps) {
            double omegaRadPerSec = velocityMps / (WHEEL_RADIUS_METERS * BALL_TRANSFER_EFFICIENCY);
            return omegaRadPerSec * (60.0 / (2.0 * Math.PI));
        }
    }
}
