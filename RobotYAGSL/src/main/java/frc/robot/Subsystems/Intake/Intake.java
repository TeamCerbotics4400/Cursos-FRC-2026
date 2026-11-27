package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final IntakeIO io;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public Command setVoltage(double voltage) {
        return run(() -> io.setVoltage(voltage));
    }

    /** Enciende el intake mientras se mantiene activo y lo apaga al terminar. */
    public Command intakeCommand() {
        return startEnd(
                () -> io.setVoltage(IntakeConstants.INTAKE_VOLTAGE),
                () -> io.setVoltage(0.0));
    }

    public Command ejectCommand() {
        return startEnd(
                () -> io.setVoltage(-IntakeConstants.INTAKE_VOLTAGE),
                () -> io.setVoltage(0.0));
    }

    public boolean consumeBall() {
        return io.consumeBall();
    }

    public int getFuelCount() {
        return io.getIntakeInputs().fuelCount;
    }

    public boolean hasFuel() {
        return getFuelCount() > 0;
    }

    public void setFuelCount(int count) {
        io.setFuelCount(count);
    }

    @Override
    public void periodic() {
        io.update();

        SmartDashboard.putNumber("Intake/Motor Voltage", io.getIntakeInputs().intakeVoltage);
        SmartDashboard.putNumber("Intake/Motor Current", io.getIntakeInputs().intakeCurrent);
        SmartDashboard.putBoolean("Intake/is Able To Intake?", io.getIntakeInputs().isAbleToIntake);
        SmartDashboard.putNumber("Intake/Fuel Count on Robot", io.getIntakeInputs().fuelCount);
    }
}
