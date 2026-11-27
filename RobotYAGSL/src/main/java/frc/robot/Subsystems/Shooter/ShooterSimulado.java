package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.Constants.ShooterConstants;
import yams.gearing.GearBox;

public class ShooterSimulado implements ShooterIO {

    TalonFX shooterIzqSuperior = new TalonFX(ShooterConstants.upLeft);
    TalonFX shooterIzqInferior = new TalonFX(ShooterConstants.downLeft);
    TalonFX shooterDerSuperior = new TalonFX(ShooterConstants.upRight);
    TalonFX shooterDerInferior = new TalonFX(ShooterConstants.downRight);

    TalonFXSimState shooterIzqSuperior_sim;
    TalonFXSimState shooterIzqInferior_sim;

    TalonFXSimState shooterDerSuperior_sim;
    TalonFXSimState shooterDerInferior_sim;

    private DCMotor gearBox = DCMotor.getKrakenX60(4);

    ShooterInputs inputs = new ShooterInputs();

    VelocityVoltage velocityOutReq = new VelocityVoltage(0);
    VoltageOut voltageOutReq = new VoltageOut(0);

    private LinearSystem<N1, N1, N1> plant = 
    LinearSystemId.createFlywheelSystem
    (gearBox, ShooterConstants.MOI, ShooterConstants.gearRatio);

    private FlywheelSim flywheelSim = new FlywheelSim(plant, gearBox);

    private TalonFXConfiguration conf = new TalonFXConfiguration();

        private SysIdRoutine sysIdRoutine;


    public ShooterSimulado(){
        shooterIzqSuperior_sim = shooterIzqSuperior.getSimState();
        shooterIzqInferior_sim = shooterIzqInferior.getSimState();
        shooterDerSuperior_sim = shooterDerSuperior.getSimState();
        shooterDerInferior_sim = shooterDerInferior.getSimState();

        shooterIzqSuperior_sim.setMotorType(MotorType.KrakenX60);

        conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        conf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        conf.Slot0.kP = ShooterConstants.kP;
        conf.Slot0.kI = ShooterConstants.kI;
        conf.Slot0.kD = ShooterConstants.kD;
        conf.Slot0.kA = ShooterConstants.kA;
        conf.Slot0.kS = ShooterConstants.kS;
        conf.Slot0.kV = ShooterConstants.kV;

        shooterIzqSuperior.getConfigurator().apply(conf);   
    
        shooterIzqInferior.setControl(new Follower(shooterIzqSuperior.getDeviceID(), MotorAlignmentValue.Aligned));
        shooterDerSuperior.setControl(new Follower(shooterIzqSuperior.getDeviceID(), MotorAlignmentValue.Opposed));
        shooterDerInferior.setControl(new Follower(shooterIzqSuperior.getDeviceID(), MotorAlignmentValue.Opposed));

        

    }

    @Override
    public void setRPMs(double rpms) {
        shooterIzqSuperior.setControl(velocityOutReq.withVelocity(rpms));
    }

    @Override
    public void setVoltage(double volts) {
        shooterIzqSuperior.setControl(new VoltageOut(volts));
    }

    @Override
    public ShooterInputs getShooterInputs() {
        return inputs;
    }

    public void simulationPeriodic() {

   // set the supply voltage of the TalonFX
   shooterIzqSuperior_sim.setSupplyVoltage(RobotController.getBatteryVoltage());

   // get the motor voltage of the TalonFX
   var motorVoltage = shooterIzqSuperior_sim.getMotorVoltageMeasure();

   flywheelSim.setInputVoltage(motorVoltage.in(Volts));
   flywheelSim.update(0.020); // assume 20 ms loop time

   shooterIzqSuperior_sim.setRotorVelocity(flywheelSim.getAngularVelocity().times(ShooterConstants.gearRatio));
}


    @Override
    public void update() {
        simulationPeriodic();

        inputs.shooterCurrent = shooterIzqSuperior.getSupplyCurrent().getValueAsDouble();
        inputs.shooterVoltage = shooterIzqSuperior.getMotorVoltage().getValueAsDouble();
        inputs.velocity = flywheelSim.getAngularVelocityRPM();
    }
    

}
