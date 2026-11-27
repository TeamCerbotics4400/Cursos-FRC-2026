package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.utils.Constants.IntakeConstants;

public class IntakeFisico implements IntakeIO {

    TalonFX motorIntake = new TalonFX(IntakeConstants.intakeID);
    IntakeInputs inputs = new IntakeInputs();

    public IntakeFisico(){

    }

    @Override
    public void setVoltage(double voltage) {
        motorIntake.setControl(new VoltageOut(voltage));
    }

    @Override
    public IntakeInputs getIntakeInputs() {
        return inputs;
    }

    @Override
    public void update() {
        inputs.intakeVoltage = motorIntake.getMotorVoltage().getValueAsDouble();
        inputs.intakeCurrent = motorIntake.getSupplyCurrent().getValueAsDouble();
        inputs.isAbleToIntake = inputs.intakeVoltage > 1.0;
    }
    
}
