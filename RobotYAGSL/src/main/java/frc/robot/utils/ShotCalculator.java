package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.ShooterConstants;

/**
 * Calcula los parámetros de disparo estático y Shoot-on-the-Move (SoTM)
 * usando tablas de interpolación (Distancia -> RPM, Ángulo de Hood y Tiempo de Vuelo)
 * junto con compensación iterativa de "Hub Virtual".
 */
public class ShotCalculator {

    public record ShotSolution(
            Rotation2d robotHeading,
            double headingVelocityRadPerSec,
            double shooterRPM,
            double hoodAngleDeg,
            double effectiveDistanceMeters,
            double timeOfFlightSec,
            Translation2d virtualTarget,
            boolean isValid) {}

    private static final InterpolatingDoubleTreeMap DISTANCE_TO_RPM = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap DISTANCE_TO_HOOD_DEG = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap DISTANCE_TO_TOF_SEC = new InterpolatingDoubleTreeMap();

    private static final double MIN_SHOOT_DISTANCE_M = 0.9;
    private static final double MAX_SHOOT_DISTANCE_M = 8.5;
    private static final int SOTM_ITERATIONS = 4;

    private static final StructPublisher<Pose2d> virtualHubPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("ShotCalculator/VirtualHubPose", Pose2d.struct)
            .publish();

    static {
        // Puntos base calibrados con la física de FuelSim (z0 = 0.55m -> Hub z = 1.83m cayendo limpio).
        // En el robot real basta con ajustar estos puntos empíricamente por distancia.
        // Formato: addPoint(distanciaMetros, rpm, hoodAngleGrados, tiempoDeVueloSegundos)
        addShotPoint(1.0, 2035.0, 79.0, 0.91);
        addShotPoint(1.5, 2105.0, 74.8, 0.93);
        addShotPoint(2.0, 2190.0, 71.2, 0.96);
        addShotPoint(2.5, 2280.0, 68.0, 1.00);
        addShotPoint(3.0, 2380.0, 65.6, 1.04);
        addShotPoint(3.5, 2480.0, 63.6, 1.08);
        addShotPoint(4.0, 2585.0, 62.1, 1.13);
        addShotPoint(5.0, 2790.0, 60.1, 1.23);
        addShotPoint(6.0, 2980.0, 58.5, 1.32);
        addShotPoint(7.5, 3270.0, 56.5, 1.44);
    }

    private static void addShotPoint(double distanceM, double rpm, double hoodDeg, double tofSec) {
        DISTANCE_TO_RPM.put(distanceM, rpm);
        DISTANCE_TO_HOOD_DEG.put(distanceM, hoodDeg);
        DISTANCE_TO_TOF_SEC.put(distanceM, tofSec);
    }

    /** Devuelve el centro del Hub correspondiente a la alianza actual. */
    public static Translation2d getAllianceHubCenter() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        return isRed ? FieldConstants.RED_HUB_CENTER : FieldConstants.BLUE_HUB_CENTER;
    }

    /**
     * Calcula la solución de tiro (estático o en movimiento).
     *
     * @param robotPose Pose actual del robot en el campo
     * @param fieldRelativeSpeeds Velocidades del chasis relativas al campo (vx, vy, omega)
     * @param enableSoTM true para compensar movimiento del chasis (Virtual Hub), false para tiro estático
     */
    public static ShotSolution calculate(
            Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, boolean enableSoTM) {
        Translation2d hubCenter = getAllianceHubCenter();
        Translation2d robotPos = robotPose.getTranslation();

        Translation2d toHub = hubCenter.minus(robotPos);
        double rawDistance = toHub.getNorm();

        double vx = enableSoTM ? fieldRelativeSpeeds.vxMetersPerSecond : 0.0;
        double vy = enableSoTM ? fieldRelativeSpeeds.vyMetersPerSecond : 0.0;

        Translation2d virtualTarget = hubCenter;
        double effectiveDistance = rawDistance;
        double tof = DISTANCE_TO_TOF_SEC.get(rawDistance);

        if (enableSoTM) {
            for (int i = 0; i < SOTM_ITERATIONS; i++) {
                virtualTarget = new Translation2d(
                        hubCenter.getX() - vx * tof,
                        hubCenter.getY() - vy * tof);
                effectiveDistance = virtualTarget.minus(robotPos).getNorm();
                tof = DISTANCE_TO_TOF_SEC.get(effectiveDistance);
            }
        }

        Translation2d toVirtual = virtualTarget.minus(robotPos);
        Rotation2d targetHeading =
                effectiveDistance > 1e-4 ? toVirtual.getAngle() : robotPose.getRotation();

        // Feedforward de velocidad angular: d/dt atan2(ry, rx) cuando el robot se traslada
        double omegaFeedforward = 0.0;
        double distSq = toVirtual.getSquaredNorm();
        if (enableSoTM && distSq > 0.05) {
            omegaFeedforward = (toVirtual.getY() * vx - toVirtual.getX() * vy) / distSq;
        }

        double clampedDist = MathUtil.clamp(effectiveDistance, MIN_SHOOT_DISTANCE_M, MAX_SHOOT_DISTANCE_M);
        double targetRPM = DISTANCE_TO_RPM.get(clampedDist);
        double targetHoodDeg = MathUtil.clamp(
                DISTANCE_TO_HOOD_DEG.get(clampedDist),
                ShooterConstants.MIN_HOOD_ANGLE_DEG,
                ShooterConstants.MAX_HOOD_ANGLE_DEG);

        boolean isValid = effectiveDistance >= MIN_SHOOT_DISTANCE_M
                && effectiveDistance <= MAX_SHOOT_DISTANCE_M;

        // Publicar telemetría para AdvantageScope / SmartDashboard
        virtualHubPublisher.set(new Pose2d(virtualTarget, targetHeading));
        SmartDashboard.putNumber("ShotCalculator/RawDistanceM", rawDistance);
        SmartDashboard.putNumber("ShotCalculator/EffectiveDistanceM", effectiveDistance);
        SmartDashboard.putNumber("ShotCalculator/TargetRPM", targetRPM);
        SmartDashboard.putNumber("ShotCalculator/TargetHoodDeg", targetHoodDeg);
        SmartDashboard.putNumber("ShotCalculator/TimeOfFlightSec", tof);
        SmartDashboard.putNumber("ShotCalculator/HeadingFFRadPerSec", omegaFeedforward);
        SmartDashboard.putBoolean("ShotCalculator/IsValidShot", isValid);
        SmartDashboard.putBoolean("ShotCalculator/SoTMEnabled", enableSoTM);

        return new ShotSolution(
                targetHeading,
                omegaFeedforward,
                targetRPM,
                targetHoodDeg,
                effectiveDistance,
                tof,
                virtualTarget,
                isValid);
    }
}
