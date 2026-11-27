package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.Constants.ShooterConstants;

public class ShooterSimulado implements ShooterIO {

    private final TalonFX shooterIzqSuperior = new TalonFX(ShooterConstants.upLeft);
    private final TalonFX shooterIzqInferior = new TalonFX(ShooterConstants.downLeft);
    private final TalonFX shooterDerSuperior = new TalonFX(ShooterConstants.upRight);
    private final TalonFX shooterDerInferior = new TalonFX(ShooterConstants.downRight);

    private final TalonFXSimState shooterIzqSuperior_sim;

    private final DCMotor gearBox = DCMotor.getKrakenX60(4);

    private final ShooterInputs inputs = new ShooterInputs();

    private final VelocityVoltage velocityOutReq = new VelocityVoltage(0);
    private final VoltageOut voltageOutReq = new VoltageOut(0);

    private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
            gearBox, ShooterConstants.MOI, ShooterConstants.gearRatio);

    private final FlywheelSim flywheelSim = new FlywheelSim(plant, gearBox);

    // Simulación dinámica del Hood variable
    private final DCMotor hoodMotorModel = DCMotor.getKrakenX60(1);
    private final DCMotorSim hoodSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    hoodMotorModel, ShooterConstants.HOOD_MOI, ShooterConstants.HOOD_GEAR_RATIO),
            hoodMotorModel);
    private final PIDController hoodPID =
            new PIDController(ShooterConstants.HOOD_KP, 0.0, ShooterConstants.HOOD_KD);

    private final TalonFXConfiguration conf = new TalonFXConfiguration();

    public ShooterSimulado() {
        shooterIzqSuperior_sim = shooterIzqSuperior.getSimState();
        shooterIzqSuperior_sim.setMotorType(MotorType.KrakenX60);

        // Coast permite que el flywheel desacelere libremente al pedir 0 RPM, igual que en un shooter real
        conf.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        conf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        conf.Slot0.kP = ShooterConstants.kP;
        conf.Slot0.kI = ShooterConstants.kI;
        conf.Slot0.kD = ShooterConstants.kD;
        conf.Slot0.kA = ShooterConstants.kA;
        conf.Slot0.kS = ShooterConstants.kS;
        conf.Slot0.kV = ShooterConstants.kV;

        shooterIzqSuperior.getConfigurator().apply(conf);

        shooterIzqInferior.setControl(
                new Follower(shooterIzqSuperior.getDeviceID(), MotorAlignmentValue.Aligned));
        shooterDerSuperior.setControl(
                new Follower(shooterIzqSuperior.getDeviceID(), MotorAlignmentValue.Opposed));
        shooterDerInferior.setControl(
                new Follower(shooterIzqSuperior.getDeviceID(), MotorAlignmentValue.Opposed));

        // Inicializar hood en su ángulo por defecto
        hoodSim.setState(Math.toRadians(ShooterConstants.DEFAULT_HOOD_ANGLE_DEG), 0.0);
    }

    @Override
    public void setRPMs(double rpms) {
        inputs.targetRPM = Math.max(0.0, rpms);
        if (inputs.targetRPM <= 1.0) {
            shooterIzqSuperior.setControl(voltageOutReq.withOutput(0.0));
        } else {
            // Phoenix 6 VelocityVoltage recibe Rotaciones Por Segundo (RPS = RPM / 60)
            double targetRPS = (inputs.targetRPM / 60.0) * ShooterConstants.gearRatio;
            shooterIzqSuperior.setControl(velocityOutReq.withVelocity(targetRPS));
        }
    }

    @Override
    public void setVoltage(double volts) {
        inputs.targetRPM = 0.0;
        shooterIzqSuperior.setControl(voltageOutReq.withOutput(volts));
    }

    @Override
    public void setHoodAngleDeg(double angleDeg) {
        inputs.targetHoodAngleDeg = MathUtil.clamp(
                angleDeg, ShooterConstants.MIN_HOOD_ANGLE_DEG, ShooterConstants.MAX_HOOD_ANGLE_DEG);
    }

    @Override
    public ShooterInputs getShooterInputs() {
        return inputs;
    }

    private void simulationPeriodic() {
        double dt = 0.020;

        // 1. Actualizar simulación del Flywheel
        shooterIzqSuperior_sim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = shooterIzqSuperior_sim.getMotorVoltageMeasure();

        flywheelSim.setInputVoltage(motorVoltage.in(Volts));
        flywheelSim.update(dt);

        double rotorRPS = (flywheelSim.getAngularVelocityRPM() / 60.0) * ShooterConstants.gearRatio;
        shooterIzqSuperior_sim.setRotorVelocity(rotorRPS);
        shooterIzqSuperior_sim.addRotorPosition(rotorRPS * dt);

        // 2. Actualizar simulación del Hood variable
        double currentHoodRad = hoodSim.getAngularPositionRad();
        double targetHoodRad = Math.toRadians(inputs.targetHoodAngleDeg);
        double hoodVolts = MathUtil.clamp(
                hoodPID.calculate(currentHoodRad, targetHoodRad),
                -RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage());

        hoodSim.setInputVoltage(hoodVolts);
        hoodSim.update(dt);

        double minRad = Math.toRadians(ShooterConstants.MIN_HOOD_ANGLE_DEG);
        double maxRad = Math.toRadians(ShooterConstants.MAX_HOOD_ANGLE_DEG);
        if (hoodSim.getAngularPositionRad() < minRad) {
            hoodSim.setState(minRad, 0.0);
        } else if (hoodSim.getAngularPositionRad() > maxRad) {
            hoodSim.setState(maxRad, 0.0);
        }
    }

    @Override
    public void update() {
        simulationPeriodic();

        inputs.shooterCurrent = shooterIzqSuperior.getSupplyCurrent().getValueAsDouble();
        inputs.shooterVoltage = shooterIzqSuperior.getMotorVoltage().getValueAsDouble();
        inputs.velocity = flywheelSim.getAngularVelocityRPM();
        inputs.isAtVelocitySetpoint = inputs.targetRPM > 1.0
                && Math.abs(inputs.velocity - inputs.targetRPM) <= ShooterConstants.VELOCITY_TOLERANCE_RPM;

        inputs.hoodAngleDeg = Math.toDegrees(hoodSim.getAngularPositionRad());
        inputs.isHoodAtSetpoint = Math.abs(inputs.hoodAngleDeg - inputs.targetHoodAngleDeg)
                <= ShooterConstants.HOOD_TOLERANCE_DEG;
    }
}
