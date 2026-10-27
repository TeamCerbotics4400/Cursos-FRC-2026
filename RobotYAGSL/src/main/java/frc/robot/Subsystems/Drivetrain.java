package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.parser.SwerveParser.SwerveDriveDevices;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Drivetrain extends SubsystemBase{

    private final PathConstraints PATHFINDING_CONSTRAINTS = 
    new PathConstraints(MetersPerSecond.of(3.0), MetersPerSecondPerSecond.of(3.0), 
    DegreesPerSecond.of(540), DegreesPerSecondPerSecond.of(540));
    
    SwerveDrive drive;
    Pigeon2 gyro;

    public Drivetrain(){
        var config = new SwerveDriveConfig()
        .withStartingPose(new Pose2d(3, 3, Rotation2d.kZero))
        .withSubsystem(this)
        .withTranslationController(new PIDController(4, 0, 0))
        .withRotationController(new PIDController(3, 0, 0))
        .withTelemetry("swerve Telemetry", TelemetryVerbosity.HIGH);

        if(RobotBase.isReal()){
                    SwerveParser.parse(new File(Filesystem.getDeployDirectory(), "swerve-config"));
        }
        else{
                    SwerveParser.parse(new File(Filesystem.getDeployDirectory(), "swerve-config-sim"));

        }

        SwerveDriveDevices devices = SwerveParser.createSwerveDriveDevices(config);

        gyro = (Pigeon2) devices.gyro();
        drive = devices.swerveDrive();

        configurePathPlanner();
    }

    public void configurePathPlanner(){

        try{
            var config = RobotConfig.fromGUISettings();
        AutoBuilder.configure(
            drive::getPose,
            drive::resetOdometry,
            drive::getRobotRelativeSpeed,
            (speeds, feedforwards) -> drive.setRobotRelativeChassisSpeeds(speeds, feedforwards.linearForces()),
            new PPHolonomicDriveController(new PIDConstants(5.0), new PIDConstants(5.0)),
                config, 
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
                this
        );
        } catch(Exception e){
            throw new RuntimeException();
        }

        
    }


    public Command drive(SwerveInputStream stream){
        return drive.drive(() -> ChassisSpeeds.fromFieldRelativeSpeeds(stream.get(), new Rotation2d(gyro.getYaw().getValue())));
    }

    public SwerveInputStream getAngularVelocityStream(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot){
        return new SwerveInputStream(drive, x, y, rot);
    }

    public Command driveToPointPathplanner(Pose2d point){
        return AutoBuilder.pathfindToPose(point, PATHFINDING_CONSTRAINTS, 0);
    }

    public Command driveToPointYAMS(Pose2d pose){
        return drive.driveToPose(pose);
    }

    public Command zeroGyro(){
        return Commands.runOnce(drive::zeroGyro, this);
    }

    public void addVisionMeasurement(Pose2d poseCalculadaPorVision, double timestamp){
        drive.addVisionMeasurement(poseCalculadaPorVision, timestamp);
    }

    public SwerveDrivePoseEstimator createPoseEstimator(){
        return new SwerveDrivePoseEstimator(drive.getKinematics(), gyro.getRotation2d(), drive.getModulePositions(), drive.getConfig().getInitialPose());
    }

    public void updatePoseEstimator(SwerveDrivePoseEstimator visionPoseEstimator){
        visionPoseEstimator.update(gyro.getRotation2d(), drive.getModulePositions());
    }

    @Override
    public void simulationPeriodic(){
        drive.simIterate();
    }

    public Pose2d getSimPose(){
        return drive.getSimPose();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        drive.updateTelemetry();
    }

}