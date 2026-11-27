package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.DrivetrainConstants;
import swervelib.parser.SwerveParser;
import swervelib.parser.SwerveParser.SwerveDriveDevices;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Drivetrain extends SubsystemBase {

    private final PathConstraints PATHFINDING_CONSTRAINTS = new PathConstraints(
            MetersPerSecond.of(3.0),
            MetersPerSecondPerSecond.of(3.0),
            DegreesPerSecond.of(540),
            DegreesPerSecondPerSecond.of(540));

    SwerveDrive drive;
    Pigeon2 gyro;

    // Controlador dedicado para apuntar al Hub / Shoot-on-the-Move
    private final PIDController aimController = new PIDController(
            DrivetrainConstants.AIM_KP,
            DrivetrainConstants.AIM_KI,
            DrivetrainConstants.AIM_KD);

    private Rotation2d targetAimHeading = Rotation2d.kZero;
    private double headingErrorDeg = 180.0;
    private boolean aimingActive = false;

    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Drivetrain/Pose", Pose2d.struct)
            .publish();

    public Drivetrain() {
        var config = new SwerveDriveConfig()
                .withStartingPose(new Pose2d(2.5, 4.02, Rotation2d.kZero))
                .withSubsystem(this)
                .withTranslationController(new PIDController(4, 0, 0))
                .withRotationController(new PIDController(3, 0, 0))
                .withTelemetry("swerve Telemetry", TelemetryVerbosity.HIGH);

        if (RobotBase.isReal()) {
            SwerveParser.parse(new File(Filesystem.getDeployDirectory(), "swerve-config"));
        } else {
            SwerveParser.parse(new File(Filesystem.getDeployDirectory(), "swerve-config-sim"));
        }

        SwerveDriveDevices devices = SwerveParser.createSwerveDriveDevices(config);

        gyro = (Pigeon2) devices.gyro();
        drive = devices.swerveDrive();

        aimController.enableContinuousInput(-Math.PI, Math.PI);

        configurePathPlanner();
    }

    public void configurePathPlanner() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    drive::getPose,
                    drive::resetOdometry,
                    drive::getRobotRelativeSpeed,
                    (speeds, feedforwards) -> drive.setRobotRelativeChassisSpeeds(speeds, feedforwards.linearForces()),
                    new PPHolonomicDriveController(new PIDConstants(5.0), new PIDConstants(5.0)),
                    config,
                    () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                            == DriverStation.Alliance.Red,
                    this);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public Command drive(SwerveInputStream stream) {
        return drive.drive(() -> ChassisSpeeds.fromFieldRelativeSpeeds(
                stream.get(), getPose().getRotation()));
    }

    /**
     * Conduce con traslación del piloto (vía {@code translationStream}) mientras orienta
     * automáticamente el chasis hacia {@code headingSupplier} con velocidad angular feedforward
     * {@code omegaFeedforwardRadPerSec} (clave para Shoot-on-the-Move sin atraso).
     */
    public Command driveAimingAtHeading(
            SwerveInputStream translationStream,
            Supplier<Rotation2d> headingSupplier,
            DoubleSupplier omegaFeedforwardRadPerSec) {
        return drive.drive(() -> {
                    ChassisSpeeds driverFieldSpeeds = translationStream.get();
                    Rotation2d currentHeading = getPose().getRotation();
                    targetAimHeading = headingSupplier.get();

                    double pidOmega = aimController.calculate(
                            currentHeading.getRadians(), targetAimHeading.getRadians());
                    double totalOmega = MathUtil.clamp(
                            pidOmega + omegaFeedforwardRadPerSec.getAsDouble(),
                            -DrivetrainConstants.MAX_AIM_ANGULAR_VEL_RAD_PER_SEC,
                            DrivetrainConstants.MAX_AIM_ANGULAR_VEL_RAD_PER_SEC);

                    double errorRad = MathUtil.angleModulus(
                            targetAimHeading.getRadians() - currentHeading.getRadians());
                    headingErrorDeg = Math.abs(Math.toDegrees(errorRad));

                    ChassisSpeeds aimedFieldSpeeds = new ChassisSpeeds(
                            driverFieldSpeeds.vxMetersPerSecond,
                            driverFieldSpeeds.vyMetersPerSecond,
                            totalOmega);

                    return ChassisSpeeds.fromFieldRelativeSpeeds(aimedFieldSpeeds, currentHeading);
                })
                .beforeStarting(() -> {
                    aimController.reset();
                    aimingActive = true;
                    headingErrorDeg = 180.0;
                })
                .finallyDo(() -> aimingActive = false);
    }

    /** Sobrecarga para apuntado estático sin feedforward angular. */
    public Command driveAimingAtHeading(
            SwerveInputStream translationStream, Supplier<Rotation2d> headingSupplier) {
        return driveAimingAtHeading(translationStream, headingSupplier, () -> 0.0);
    }

    /** Verdadero si el comando de apuntado está activo y el error angular está dentro de tolerancia. */
    public boolean isAimed() {
        return aimingActive && headingErrorDeg <= DrivetrainConstants.AIM_TOLERANCE_DEG;
    }

    /** Verifica si el chasis mira hacia un ángulo dado dentro de tolerancia (útil también en autónomo). */
    public boolean isFacingHeading(Rotation2d desiredHeading) {
        double errorDeg = Math.abs(desiredHeading.minus(getPose().getRotation()).getDegrees());
        return errorDeg <= DrivetrainConstants.AIM_TOLERANCE_DEG;
    }

    public SwerveInputStream getAngularVelocityStream(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return new SwerveInputStream(drive, x, y, rot);
    }

    public Command driveToPointPathplanner(Pose2d point) {
        return AutoBuilder.pathfindToPose(point, PATHFINDING_CONSTRAINTS, 0);
    }

    public Command driveToPointYAMS(Pose2d pose) {
        return drive.driveToPose(pose);
    }

    public Command zeroGyro() {
        return Commands.runOnce(drive::zeroGyro, this);
    }

    public void addVisionMeasurement(Pose2d poseCalculadaPorVision, double timestamp) {
        drive.addVisionMeasurement(poseCalculadaPorVision, timestamp);
    }

    public SwerveDrivePoseEstimator createPoseEstimator() {
        return new SwerveDrivePoseEstimator(
                drive.getKinematics(),
                gyro.getRotation2d(),
                drive.getModulePositions(),
                drive.getConfig().getInitialPose());
    }

    public void updatePoseEstimator(SwerveDrivePoseEstimator visionPoseEstimator) {
        visionPoseEstimator.update(gyro.getRotation2d(), drive.getModulePositions());
    }

    @Override
    public void simulationPeriodic() {
        drive.simIterate();
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return drive.getFieldRelativeSpeed();
    }

    public Pose2d getSimPose() {
        return drive.getSimPose();
    }

    @Override
    public void periodic() {
        drive.updateTelemetry();

        Pose2d currentPose = getPose();
        posePublisher.set(currentPose);
        SmartDashboard.putNumber("Drivetrain/HeadingDeg", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Drivetrain/TargetHeadingDeg", targetAimHeading.getDegrees());
        SmartDashboard.putNumber("Drivetrain/HeadingErrorDeg", headingErrorDeg);
        SmartDashboard.putBoolean("Drivetrain/IsAimed", isAimed());
    }
}
