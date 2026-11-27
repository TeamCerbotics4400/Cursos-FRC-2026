// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MilliOhms;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeFisico;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeSimulado;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterFisico;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterSimulado;
import frc.robot.utils.Constants.DrivetrainConstants;
import frc.robot.utils.simulation.FuelSim;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.simulation.BatterySim;

public class RobotContainer {

  Drivetrain drivetrain = new Drivetrain();
  Intake intake;
  IntakeIO intakeIO;
  Shooter shooter;
  ShooterIO shooterIO;

  CommandXboxController driver = new CommandXboxController(0);

  private SendableChooser<Command> autChooser;

  private boolean headingControlEnabled = false;

  private SwerveInputStream driveStream = drivetrain.getAngularVelocityStream(driver::getLeftY,
      driver::getLeftX, () -> driver.getRawAxis(4))
      .withControllerHeadingAxis(driver::getRightX, driver::getRightY)
      .withDeadband(0.05)
      .withAllianceRelativeControl();

  FuelSim fuelSim = new FuelSim("Fuel Simulation");

  private void configureBatterySim() {
    InterpolatingDoubleTreeMap wormBatteryCurve = new InterpolatingDoubleTreeMap();

    wormBatteryCurve.put(0.00, 6.0);
    wormBatteryCurve.put(0.05, 6.5);
    wormBatteryCurve.put(0.1, 9.5);
    wormBatteryCurve.put(0.2, 10.2);
    wormBatteryCurve.put(0.4, 10.6);
    wormBatteryCurve.put(0.6, 10.9);
    wormBatteryCurve.put(0.8, 11.2);

    BatterySim.replaceSOCInterpolation(wormBatteryCurve);
    BatterySim.enableDischarge(18, Volts.of(11), MilliOhms.of(20));
  }

  public RobotContainer() {

    if (RobotBase.isSimulation()) {
      fuelSim.start();
      fuelSim.spawnStartingFuel();
      configureFuelSimRobot();

      intakeIO = new IntakeSimulado(fuelSim);
      shooterIO = new ShooterSimulado();

    } else {
      intakeIO = new IntakeFisico();
      shooterIO = new ShooterFisico();
    }

    intake = new Intake(intakeIO);
    shooter = new Shooter(shooterIO);

    NamedCommands.registerCommand("Intakear", intake.setVoltage(12));

    configureBatterySim();

    autChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autChooser);

    configureBindings();
  }

  private void configureFuelSimRobot() {
    fuelSim.registerRobot(
        DrivetrainConstants.TRACKWIDTH,
        DrivetrainConstants.TRACKLENGTH,
        DrivetrainConstants.BUMPER_HEIGHT,
        drivetrain::getPose,
        drivetrain::getChassisSpeeds);

    double largoRobot = DrivetrainConstants.TRACKLENGTH;
    fuelSim.registerIntake(
        largoRobot,
        largoRobot + 0.3,
        0,
        0.3,
        () -> intakeIO.getIntakeInputs().isAbleToIntake,
        () -> intakeIO.intakedBall());
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.drive(driveStream));

    // driver.x().whileTrue(drivetrain.driveToPointPathplanner(new Pose2d(3, 3,
    // Rotation2d.kZero)));

    driver.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    driver.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    driver.y().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driver.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    driver.b().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driver.x().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    driver.leftTrigger().onTrue(shooter.setRPMs(4000))
    .onFalse(shooter.setRPMs(0));
  }

  public Command getAutonomousCommand() {
    return autChooser.getSelected();
  }
}
