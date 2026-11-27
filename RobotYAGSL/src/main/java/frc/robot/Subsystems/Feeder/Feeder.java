package frc.robot.Subsystems.Feeder;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.FeederConstants;
import frc.robot.utils.simulation.FuelSim;

public class Feeder extends SubsystemBase {

    private final FeederIO io;

    public Feeder(FeederIO io) {
        this.io = io;
    }

    /** Alimenta continuamente hacia el shooter independientemente del estado de setpoint. */
    public Command feedCommand() {
        return startEnd(
                () -> io.setVoltage(FeederConstants.FEED_VOLTAGE),
                () -> io.setVoltage(0.0));
    }

    /**
     * Alimenta hacia el shooter únicamente cuando {@code readyToShoot} es verdadero
     * (por ejemplo: flywheel en RPM + hood en ángulo + chasis apuntando al Hub).
     */
    public Command feedWhenReadyCommand(BooleanSupplier readyToShoot) {
        return run(() -> {
            if (readyToShoot.getAsBoolean()) {
                io.setVoltage(FeederConstants.FEED_VOLTAGE);
            } else {
                io.setVoltage(0.0);
            }
        }).finallyDo(() -> io.setVoltage(0.0));
    }

    public Command reverseCommand() {
        return startEnd(
                () -> io.setVoltage(-6.0),
                () -> io.setVoltage(0.0));
    }

    public Command stopCommand() {
        return runOnce(() -> io.setVoltage(0.0));
    }

    @Override
    public void periodic() {
        io.update();

        var in = io.getFeederInputs();
        SmartDashboard.putNumber("Feeder/Voltage", in.feederVoltage);
        SmartDashboard.putNumber("Feeder/Current", in.feederCurrent);
        SmartDashboard.putNumber("Feeder/VelocityRPM", in.feederVelocityRPM);
        SmartDashboard.putBoolean("Feeder/IsFeeding", in.isFeeding);
        SmartDashboard.putNumber("Feeder/TotalBallsLaunched", in.totalBallsLaunched);

        // Puntaje acumulado en los Hubs de FuelSim
        SmartDashboard.putNumber("FuelSim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
        SmartDashboard.putNumber("FuelSim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
    }
}
