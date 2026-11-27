package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final SysIdRoutine sysIdRoutine;

    public Shooter(ShooterIO io) {
        this.io = io;

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(1.0).per(Second),
                        Volts.of(7.0),
                        null,
                        state -> SignalLogger.writeString("SysIDFlywheelState", state.toString())),
                new SysIdRoutine.Mechanism(volts -> io.setVoltage(volts.in(Volts)), null, this));
    }

    /** Comando para acelerar a RPM fijas manteniendo el hood en su ángulo actual. */
    public Command setRPMs(double rpms) {
        return run(() -> io.setRPMs(rpms));
    }

    /** Comando continuo para controlar dinámicamente RPM y ángulo de Hood (ej. desde ShotCalculator). */
    public Command trackShotCommand(DoubleSupplier rpmSupplier, DoubleSupplier hoodAngleDegSupplier) {
        return run(() -> {
            io.setRPMs(rpmSupplier.getAsDouble());
            io.setHoodAngleDeg(hoodAngleDegSupplier.getAsDouble());
        }).finallyDo(() -> io.setRPMs(0.0));
    }

    /** Comando para fijar RPM y ángulo de Hood estáticos mientras esté activo y apagar al soltar. */
    public Command spinUpCommand(double rpms, double hoodAngleDeg) {
        return trackShotCommand(() -> rpms, () -> hoodAngleDeg);
    }

    public Command setHoodAngle(double angleDeg) {
        return runOnce(() -> io.setHoodAngleDeg(angleDeg));
    }

    public Command stopCommand() {
        return runOnce(() -> io.setRPMs(0.0));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command setVoltage(double voltage) {
        return run(() -> io.setVoltage(voltage));
    }

    public boolean atVelocitySetpoint() {
        return io.getShooterInputs().isAtVelocitySetpoint;
    }

    public boolean isHoodAtSetpoint() {
        return io.getShooterInputs().isHoodAtSetpoint;
    }

    /** Verdadero cuando tanto las RPM del flywheel como el ángulo del hood están en tolerancia. */
    public boolean isReadyToShoot() {
        return atVelocitySetpoint() && isHoodAtSetpoint();
    }

    public double getVelocityRPM() {
        return io.getShooterInputs().velocity;
    }

    public double getTargetRPM() {
        return io.getShooterInputs().targetRPM;
    }

    public double getHoodAngleDeg() {
        return io.getShooterInputs().hoodAngleDeg;
    }

    /** Velocidad lineal real de salida de la pelota (m/s) según las RPM actuales del flywheel. */
    public double getLaunchVelocityMetersPerSec() {
        return ShooterConstants.rpmToLaunchVelocityMps(getVelocityRPM());
    }

    @Override
    public void periodic() {
        io.update();

        var in = io.getShooterInputs();
        SmartDashboard.putNumber("Shooter/Voltage", in.shooterVoltage);
        SmartDashboard.putNumber("Shooter/Current", in.shooterCurrent);
        SmartDashboard.putNumber("Shooter/Velocity", in.velocity);
        SmartDashboard.putNumber("Shooter/TargetRPM", in.targetRPM);
        SmartDashboard.putBoolean("Shooter/AtVelocitySetpoint", in.isAtVelocitySetpoint);
        SmartDashboard.putNumber("Shooter/HoodAngleDeg", in.hoodAngleDeg);
        SmartDashboard.putNumber("Shooter/TargetHoodAngleDeg", in.targetHoodAngleDeg);
        SmartDashboard.putBoolean("Shooter/AtHoodSetpoint", in.isHoodAtSetpoint);
        SmartDashboard.putBoolean("Shooter/ReadyToShoot", isReadyToShoot());
    }
}
