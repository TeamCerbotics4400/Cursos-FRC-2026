package frc.robot.Subsystems.Intake;

public interface IntakeIO {

    public class IntakeInputs {

        public double intakeVoltage = 0.0;

        public double intakeCurrent = 0.0;
        public boolean isAbleToIntake = false;

        public int fuelCount = 0;
    }

    public default void intakedBall(){
        if(getIntakeInputs().fuelCount < 60){
            getIntakeInputs().fuelCount++;
        }
    }

    public abstract void setVoltage(double voltage);

    public abstract IntakeInputs getIntakeInputs();

    public abstract void update();
    
}
