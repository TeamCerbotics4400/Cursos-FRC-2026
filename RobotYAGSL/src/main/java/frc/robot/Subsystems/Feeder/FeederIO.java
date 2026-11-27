package frc.robot.Subsystems.Feeder;

public interface FeederIO {

    public class FeederInputs {
        public double feederVoltage = 0.0;
        public double feederCurrent = 0.0;
        public double feederVelocityRPM = 0.0;
        public boolean isFeeding = false;
        public int totalBallsLaunched = 0;
    }

    public abstract void setVoltage(double voltage);

    public abstract FeederInputs getFeederInputs();

    public abstract void update();
}
