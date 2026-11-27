package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.utils.Constants.ShooterConstants;

public class ShooterFisico implements ShooterIO {

    private final TalonFX shooterIzqSuperior = new TalonFX(ShooterConstants.upLeft);
    private final TalonFX shooterIzqInferior = new TalonFX(ShooterConstants.downLeft);
    private final TalonFX shooterDerSuperior = new TalonFX(ShooterConstants.upRight);
    private final TalonFX shooterDerInferior = new TalonFX(ShooterConstants.downRight);
    private final TalonFX hoodMotor = new TalonFX(ShooterConstants.hoodID);

    private final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    private final VoltageOut voltageOutReq = new VoltageOut(0);
    private final VelocityVoltage velocityOutReq = new VelocityVoltage(0);
    private final PositionVoltage hoodPositionReq = new PositionVoltage(0);

    private final ShooterInputs inputs = new ShooterInputs();

    public ShooterFisico() {
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flywheelConfig.Slot0.kP = ShooterConstants.kP;
        flywheelConfig.Slot0.kI = ShooterConstants.kI;
        flywheelConfig.Slot0.kD = ShooterConstants.kD;
        flywheelConfig.Slot0.kA = ShooterConstants.kA;
        flywheelConfig.Slot0.kS = ShooterConstants.kS;
        flywheelConfig.Slot0.kV = ShooterConstants.kV;

        shooterIzqSuperior.getConfigurator().apply(flywheelConfig);

        shooterIzqInferior.setControl(
                new Follower(shooterIzqSuperior.getDeviceID(), MotorAlignmentValue.Aligned));
        shooterDerSuperior.setControl(
                new Follower(shooterIzqSuperior.getDeviceID(), MotorAlignmentValue.Opposed));
        shooterDerInferior.setControl(
                new Follower(shooterIzqSuperior.getDeviceID(), MotorAlignmentValue.Opposed));

        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.Feedback.SensorToMechanismRatio = ShooterConstants.HOOD_GEAR_RATIO;
        hoodConfig.Slot0.kP = ShooterConstants.HOOD_KP;
        hoodConfig.Slot0.kD = ShooterConstants.HOOD_KD;
        hoodMotor.getConfigurator().apply(hoodConfig);
        hoodMotor.setPosition(ShooterConstants.DEFAULT_HOOD_ANGLE_DEG / 360.0);
    }

    @Override
    public void setRPMs(double rpms) {
        inputs.targetRPM = Math.max(0.0, rpms);
        if (inputs.targetRPM <= 1.0) {
            shooterIzqSuperior.setControl(voltageOutReq.withOutput(0.0));
        } else {
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
        hoodMotor.setControl(hoodPositionReq.withPosition(inputs.targetHoodAngleDeg / 360.0));
    }

    @Override
    public ShooterInputs getShooterInputs() {
        return inputs;
    }

    @Override
    public void update() {
        inputs.shooterCurrent = shooterIzqSuperior.getSupplyCurrent().getValueAsDouble();
        inputs.shooterVoltage = shooterIzqSuperior.getMotorVoltage().getValueAsDouble();
        inputs.velocity =
                (shooterIzqSuperior.getVelocity().getValueAsDouble() / ShooterConstants.gearRatio) * 60.0;
        inputs.isAtVelocitySetpoint = inputs.targetRPM > 1.0
                && Math.abs(inputs.velocity - inputs.targetRPM) <= ShooterConstants.VELOCITY_TOLERANCE_RPM;

        inputs.hoodAngleDeg = hoodMotor.getPosition().getValueAsDouble() * 360.0;
        inputs.isHoodAtSetpoint = Math.abs(inputs.hoodAngleDeg - inputs.targetHoodAngleDeg)
                <= ShooterConstants.HOOD_TOLERANCE_DEG;
    }
}
