package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.utils.Constants.ShooterConstants;

public interface ShooterIO {
    public class ShooterInputs {
        public double shooterVoltage = 0.0;
        public double shooterCurrent = 0.0;
        public double velocity = 0.0; // RPM actuales del flywheel
        public double targetRPM = 0.0;
        public boolean isAtVelocitySetpoint = false;

        public double hoodAngleDeg = ShooterConstants.DEFAULT_HOOD_ANGLE_DEG;
        public double targetHoodAngleDeg = ShooterConstants.DEFAULT_HOOD_ANGLE_DEG;
        public boolean isHoodAtSetpoint = true;
    }

    public abstract void setRPMs(double rpms);

    public abstract void setVoltage(double voltage);

    public abstract void setHoodAngleDeg(double angleDeg);

    public abstract ShooterInputs getShooterInputs();

    public default void quasisticTest(Direction direction) {}

    public default void dynamicTest(Direction direction) {}

    public void update();
}
