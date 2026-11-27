package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.Constants.ShooterConstants;

public class ShooterFisico implements ShooterIO {

    TalonFX shooterIzqSuperior = new TalonFX(ShooterConstants.upLeft);
    TalonFX shooterIzqInferior = new TalonFX(ShooterConstants.downLeft);
    TalonFX shooterDerSuperior = new TalonFX(ShooterConstants.upRight);
    TalonFX shooterDerInferior = new TalonFX(ShooterConstants.downRight);

    TalonFXConfiguration leftUpConfig = new TalonFXConfiguration();
    TalonFXConfiguration leftDownConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightUpConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightDownConfig = new TalonFXConfiguration();
    
    VoltageOut voltageOutReq;
    VelocityVoltage velocityOutReq;
    VoltageOut sysIdReq; 

    ShooterInputs inputs = new ShooterInputs();



    public ShooterFisico(){
        leftUpConfig.Slot0.kP = ShooterConstants.kP;
        leftUpConfig.Slot0.kI = ShooterConstants.kI;
        leftUpConfig.Slot0.kD = ShooterConstants.kD;
        leftUpConfig.Slot0.kA = ShooterConstants.kA;
        leftUpConfig.Slot0.kS = ShooterConstants.kS;
        leftUpConfig.Slot0.kV = ShooterConstants.kV;

        shooterIzqInferior.setControl(new Follower(shooterIzqSuperior.getDeviceID(), MotorAlignmentValue.Aligned));
        shooterDerSuperior.setControl(new Follower(shooterIzqSuperior.getDeviceID(), MotorAlignmentValue.Opposed));
        shooterDerInferior.setControl(new Follower(shooterIzqSuperior.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void setRPMs(double rpms) {
        shooterIzqSuperior.setControl(velocityOutReq.withVelocity(rpms));
    }

    @Override
    public void setVoltage(double volts) {
        shooterIzqSuperior.setControl(voltageOutReq.withOutput(volts));
    }

    @Override
    public ShooterInputs getShooterInputs() {
        return inputs;
    }

    @Override
    public void update() {
        inputs.shooterCurrent = shooterIzqSuperior.getSupplyCurrent().getValueAsDouble();
        inputs.shooterVoltage = shooterIzqSuperior.getMotorVoltage().getValueAsDouble();
        inputs.velocity = shooterIzqSuperior.getVelocity().getValueAsDouble();
    }
    
}
