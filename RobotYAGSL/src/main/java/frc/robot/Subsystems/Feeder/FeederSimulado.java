package frc.robot.Subsystems.Feeder;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.Constants.FeederConstants;
import frc.robot.utils.Constants.ShooterConstants;
import frc.robot.utils.simulation.FuelSim;

public class FeederSimulado implements FeederIO {

    private final TalonFX motorFeeder = new TalonFX(FeederConstants.feederID);
    private final TalonFXSimState motorFeederSim;
    private final VoltageOut voltageReq = new VoltageOut(0);

    private final DCMotor motorModel = DCMotor.getKrakenX60(1);
    private final DCMotorSim feederPlantSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    motorModel, FeederConstants.MOI, FeederConstants.GEAR_RATIO),
            motorModel);

    private final FuelSim fuelSim;
    private final BooleanSupplier consumeFuelCallback;
    private final DoubleSupplier launchVelocityMpsSupplier;
    private final DoubleSupplier hoodAngleDegSupplier;

    private final FeederInputs inputs = new FeederInputs();
    private double lastShotTimestamp = -10.0;

    public FeederSimulado(
            FuelSim fuelSim,
            BooleanSupplier consumeFuelCallback,
            DoubleSupplier launchVelocityMpsSupplier,
            DoubleSupplier hoodAngleDegSupplier) {
        this.fuelSim = fuelSim;
        this.consumeFuelCallback = consumeFuelCallback;
        this.launchVelocityMpsSupplier = launchVelocityMpsSupplier;
        this.hoodAngleDegSupplier = hoodAngleDegSupplier;

        motorFeederSim = motorFeeder.getSimState();
        motorFeederSim.setMotorType(MotorType.KrakenX60);
    }

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
        double dt = 0.020;
        motorFeederSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double appliedVolts = motorFeederSim.getMotorVoltage();

        feederPlantSim.setInputVoltage(appliedVolts);
        feederPlantSim.update(dt);

        double rotorRPS =
                (feederPlantSim.getAngularVelocityRPM() / 60.0) * FeederConstants.GEAR_RATIO;
        motorFeederSim.setRotorVelocity(rotorRPS);
        motorFeederSim.addRotorPosition(rotorRPS * dt);

        inputs.feederVoltage = appliedVolts;
        inputs.feederCurrent = motorFeeder.getSupplyCurrent().getValueAsDouble();
        inputs.feederVelocityRPM = feederPlantSim.getAngularVelocityRPM();
        inputs.isFeeding = appliedVolts > 1.0 && inputs.feederVelocityRPM > 50.0;

        double launchSpeedMps = launchVelocityMpsSupplier.getAsDouble();
        double now = Timer.getFPGATimestamp();

        // Si el feeder está alimentando, el flywheel tiene velocidad y pasó el tiempo entre tiros:
        if (inputs.isFeeding
                && launchSpeedMps > 1.0
                && (now - lastShotTimestamp) >= FeederConstants.SHOT_PERIOD_SEC) {
            if (consumeFuelCallback.getAsBoolean()) {
                lastShotTimestamp = now;
                inputs.totalBallsLaunched++;
                fuelSim.launchFuel(
                        MetersPerSecond.of(launchSpeedMps),
                        Degrees.of(hoodAngleDegSupplier.getAsDouble()),
                        Degrees.of(0.0), // Sin torreta: apunta con el frente (+X) del chasis
                        Meters.of(ShooterConstants.LAUNCH_HEIGHT_METERS));
            }
        }
    }
}
