// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MilliOhms;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain;

import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.simulation.BatterySim;

public class RobotContainer {

  Drivetrain drivetrain = new Drivetrain();
  CommandXboxController driver = new CommandXboxController(0);

  private SendableChooser<Command> autChooser;

  private boolean headingControlEnabled = false;

  private SwerveInputStream driveStream = drivetrain.getAngularVelocityStream(driver::getLeftY,
   driver::getLeftX, () -> driver.getRawAxis(4))
   .withControllerHeadingAxis(driver::getRightX, driver::getRightY)
   .withDeadband(0.05)
   .withAllianceRelativeControl();
   

  private void configureBatterySim(){
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

    configureBatterySim();

    autChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autChooser);

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.drive(driveStream));
    
    driver.x().whileTrue(drivetrain.driveToPointPathplanner(new Pose2d(3, 3, Rotation2d.kZero)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
