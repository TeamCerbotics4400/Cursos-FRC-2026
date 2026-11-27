package frc.robot.Subsystems.Intake;

import frc.robot.utils.Constants.IntakeConstants;

public interface IntakeIO {

    public class IntakeInputs {
        public double intakeVoltage = 0.0;
        public double intakeCurrent = 0.0;
        public boolean isAbleToIntake = false;
        public int fuelCount = 0;
    }

    public default void intakedBall() {
        if (getIntakeInputs().fuelCount < IntakeConstants.MAX_FUEL_CAPACITY) {
            getIntakeInputs().fuelCount++;
        }
    }

    /** Descuenta 1 pelota almacenada si hay disponibles. Devuelve true si se consumió. */
    public default boolean consumeBall() {
        if (getIntakeInputs().fuelCount > 0) {
            getIntakeInputs().fuelCount--;
            return true;
        }
        return false;
    }

    public default void setFuelCount(int count) {
        getIntakeInputs().fuelCount =
                Math.max(0, Math.min(IntakeConstants.MAX_FUEL_CAPACITY, count));
    }

    public abstract void setVoltage(double voltage);

    public abstract IntakeInputs getIntakeInputs();

    public abstract void update();
}
