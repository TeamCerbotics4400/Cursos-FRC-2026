package frc.robot.Subsystems.Feeder;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.utils.Constants.FeederConstants;

public class FeederFisico implements FeederIO {

    private final TalonFX motorFeeder = new TalonFX(FeederConstants.feederID);
    private final VoltageOut voltageReq = new VoltageOut(0);
    private final FeederInputs inputs = new FeederInputs();

    @Override
    public void setVoltage(double voltage) {
        motorFeeder.setControl(voltageReq.withOutput(voltage));
    }

    @Override
    public FeederInputs getFeederInputs() {
        return inputs;
    }

    @Override
    public void update() {
        inputs.feederVoltage = motorFeeder.getMotorVoltage().getValueAsDouble();
        inputs.feederCurrent = motorFeeder.getSupplyCurrent().getValueAsDouble();
        inputs.feederVelocityRPM =
                (motorFeeder.getVelocity().getValueAsDouble() / FeederConstants.GEAR_RATIO) * 60.0;
        inputs.isFeeding = inputs.feederVoltage > 1.0;
    }
}
