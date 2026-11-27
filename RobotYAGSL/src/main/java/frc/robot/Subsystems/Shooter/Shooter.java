package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {

    ShooterIO io;

    private SysIdRoutine sysIdRoutine;

    public Shooter(ShooterIO io) {
        this.io = io;

        sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(
                Volts.of(1.0).per(Second),
                Volts.of(7.0), null,
                state -> SignalLogger.writeString("SysIDFlywheelState", state.toString())),
                new SysIdRoutine.Mechanism(volts -> io.setVoltage(volts.in(Volts)), null, this));
    }

    public Command setRPMs(double rpms) {
        return run(
                () -> io.setRPMs(rpms));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command setVoltage(double voltage) {
        return run(
                () -> io.setVoltage(voltage));
    }

    @Override
    public void periodic() {
        io.update();

        SmartDashboard.putNumber("Shooter/Voltage", io.getShooterInputs().shooterVoltage);
        SmartDashboard.putNumber("Shooter/Current", io.getShooterInputs().shooterCurrent);
        SmartDashboard.putNumber("Shooter/Velocity", io.getShooterInputs().velocity);
    }
}
