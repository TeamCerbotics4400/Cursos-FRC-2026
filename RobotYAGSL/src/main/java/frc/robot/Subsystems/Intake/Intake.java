package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    IntakeIO io;

    public Intake(IntakeIO io){
        this.io = io;
    }

    public Command setVoltage(double voltage){
        return run(
            () -> io.setVoltage(voltage)
        );
    }

    @Override
    public void periodic(){
        io.update();

        SmartDashboard.putNumber("Intake/Motor Voltage", io.getIntakeInputs().intakeVoltage);
        SmartDashboard.putNumber("Intake/Motor Current", io.getIntakeInputs().intakeCurrent);
        SmartDashboard.putBoolean("Intake/is Able To Intake?", io.getIntakeInputs().isAbleToIntake);
        SmartDashboard.putNumber("Intake/Fuel Count on Robot", io.getIntakeInputs().fuelCount);
    }
    
}
