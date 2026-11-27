package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public interface ShooterIO {
    public class ShooterInputs {
        public double shooterVoltage = 0.0;
        public double shooterCurrent = 0.0;
        public double velocity = 0.0;
        

    }

    public abstract void setRPMs(double rpms);
    public abstract void setVoltage(double voltage);
    public abstract ShooterInputs getShooterInputs();

    public default void quasisticTest(Direction direction) {

    };

    public default void dynamicTest(Direction direction) {};

    public void update();
    
} 
