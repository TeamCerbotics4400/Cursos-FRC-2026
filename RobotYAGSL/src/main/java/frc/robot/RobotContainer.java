// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MilliOhms;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Feeder.Feeder;
import frc.robot.Subsystems.Feeder.FeederFisico;
import frc.robot.Subsystems.Feeder.FeederIO;
import frc.robot.Subsystems.Feeder.FeederSimulado;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeFisico;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeSimulado;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterFisico;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterSimulado;
import frc.robot.utils.Constants.DrivetrainConstants;
import frc.robot.utils.Constants.ShooterConstants;
import frc.robot.utils.ShotCalculator;
import frc.robot.utils.ShotCalculator.ShotSolution;
import frc.robot.utils.simulation.FuelSim;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.simulation.BatterySim;

public class RobotContainer {

    Drivetrain drivetrain = new Drivetrain();
    Intake intake;
    IntakeIO intakeIO;
    Shooter shooter;
    ShooterIO shooterIO;
    Feeder feeder;
    FeederIO feederIO;

    CommandXboxController driver = new CommandXboxController(0);

    private SendableChooser<Command> autChooser;

    // Habilita o deshabilita la compensación Shoot-on-the-Move (Hub Virtual) en tiempo real
    private boolean sotmEnabled = true;
    private ShotSolution latestShot;

    private SwerveInputStream driveStream = drivetrain
            .getAngularVelocityStream(
                    driver::getLeftY, driver::getLeftX, () -> driver.getRawAxis(4))
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

        if (RobotBase.isSimulation()) {
            // Precargar 8 pelotas iniciales en simulación para probar tiros de inmediato
            intake.setFuelCount(8);
            feederIO = new FeederSimulado(
                    fuelSim,
                    intake::consumeBall,
                    shooter::getLaunchVelocityMetersPerSec,
                    shooter::getHoodAngleDeg);
        } else {
            feederIO = new FeederFisico();
        }

        feeder = new Feeder(feederIO);

        // Inicializar primera solución de tiro y setpoint manual en Dashboard
        SmartDashboard.putNumber("Shooter/ManualSetpointRPM", 2500.0);
        SmartDashboard.putNumber(
                "Shooter/ManualHoodDeg", ShooterConstants.DEFAULT_HOOD_ANGLE_DEG);
        updateShotCalculation();

        NamedCommands.registerCommand("Intakear", intake.intakeCommand());
        NamedCommands.registerCommand("ApuntarYDisparar", buildAimAndShootCommand());

        configureBatterySim();

        autChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autChooser);

        configureBindings();
    }

    /** Se llama cada ciclo desde Robot.robotPeriodic() para actualizar ShotCalculator. */
    public void updateShotCalculation() {
        latestShot = ShotCalculator.calculate(
                drivetrain.getPose(), drivetrain.getChassisSpeeds(), sotmEnabled);
    }

    private void configureFuelSimRobot() {
        fuelSim.registerRobot(
                DrivetrainConstants.TRACKWIDTH,
                DrivetrainConstants.TRACKLENGTH,
                DrivetrainConstants.BUMPER_HEIGHT,
                drivetrain::getPose,
                drivetrain::getChassisSpeeds);

        // Coordenadas relativas al centro del robot: justo frente al bumper delantero (+X)
        double halfLength = DrivetrainConstants.TRACKLENGTH / 2.0;
        double halfWidth = DrivetrainConstants.TRACKWIDTH / 2.0;
        fuelSim.registerIntake(
                halfLength,
                halfLength + 0.30,
                -halfWidth - 0.05,
                halfWidth + 0.05,
                () -> intakeIO.getIntakeInputs().isAbleToIntake,
                () -> intakeIO.intakedBall());
    }

    /**
     * Comando completo de tiro al Hub (funciona tanto quieto como en movimiento con SoTM):
     * 1. Apunta el chasis al Hub (o Hub Virtual si SoTM está activo).
     * 2. Acelera el Shooter y posiciona el Hood según la distancia calculada.
     * 3. Activa el Feeder automáticamente solo cuando RPM, Hood y ángulo del chasis están en setpoint.
     */
    public Command buildAimAndShootCommand() {
        return Commands.parallel(
                drivetrain.driveAimingAtHeading(
                        driveStream,
                        () -> latestShot.robotHeading(),
                        () -> latestShot.headingVelocityRadPerSec()),
                shooter.trackShotCommand(
                        () -> latestShot.shooterRPM(),
                        () -> latestShot.hoodAngleDeg()),
                feeder.feedWhenReadyCommand(
                        () -> shooter.isReadyToShoot()
                                && drivetrain.isAimed()
                                && latestShot.isValid()));
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.drive(driveStream));

        // --- CONTROLES PRINCIPALES DE JUEGO ---

        // Gatillo Izquierdo (LT): Recoger pelotas con el Intake
        driver.leftTrigger().whileTrue(intake.intakeCommand());

        // Bumper Derecho (RB): Botón dedicado para apuntar el robot al Hub mientras manejas
        driver.rightBumper().whileTrue(drivetrain.driveAimingAtHeading(
                driveStream,
                () -> latestShot.robotHeading(),
                () -> latestShot.headingVelocityRadPerSec()));

        // Gatillo Derecho (RT): Apuntar al Hub + Acelerar Shooter/Hood + Disparar al llegar a setpoint
        driver.rightTrigger().whileTrue(buildAimAndShootCommand());

        // Bumper Izquierdo (LB): Probar aceleración del Shooter a un setpoint manual configurable en Dashboard
        driver.leftBumper().whileTrue(shooter.trackShotCommand(
                () -> SmartDashboard.getNumber("Shooter/ManualSetpointRPM", 2500.0),
                () -> SmartDashboard.getNumber(
                        "Shooter/ManualHoodDeg", ShooterConstants.DEFAULT_HOOD_ANGLE_DEG)));

        // Botón A: Alimentar Feeder hacia el Shooter (dispara cuando el Shooter llegó a su setpoint)
        driver.a().whileTrue(feeder.feedWhenReadyCommand(shooter::isReadyToShoot));

        // Botón B: Alternar compensación Shoot-on-the-Move (SoTM ON / OFF)
        driver.b().onTrue(Commands.runOnce(() -> sotmEnabled = !sotmEnabled));

        // Botón X (Simulación): Recargar +10 pelotas en el inventario del robot para pruebas rápidas
        if (RobotBase.isSimulation()) {
            driver.x().onTrue(Commands.runOnce(
                    () -> intake.setFuelCount(intake.getFuelCount() + 10)));
        }

        // --- HERRAMIENTAS DE CARACTERIZACIÓN (SysId y SignalLogger) ---
        driver.start().onTrue(Commands.runOnce(SignalLogger::start));
        driver.back().onTrue(Commands.runOnce(SignalLogger::stop));

        driver.povUp().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driver.povDown().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        driver.povRight().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driver.povLeft().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autChooser.getSelected();
    }
}
