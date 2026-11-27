package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.simulation.FuelSim;

public class IntakeSimulado implements IntakeIO {
    TalonFX motorIntake = new TalonFX(IntakeConstants.intakeID);
    TalonFXSimState motorIntakeSim;

    FuelSim fuelSim;

    IntakeInputs inputs = new IntakeInputs();

    public IntakeSimulado(FuelSim fuelSim){
        this.fuelSim = fuelSim;
        motorIntakeSim = motorIntake.getSimState();
    }


    @Override
    public void setVoltage(double voltage) {
        motorIntake.setControl(new VoltageOut(voltage));
    }

    @Override
    public IntakeInputs getIntakeInputs() {
        // TODO Auto-generated method stub
        return inputs;
    }

    @Override
    public void update() {
        inputs.intakeVoltage = motorIntakeSim.getMotorVoltage();
        inputs.intakeCurrent = motorIntakeSim.getSupplyCurrent();

        inputs.isAbleToIntake = inputs.intakeVoltage > 1.0  && inputs.fuelCount < 60;
    }
}
