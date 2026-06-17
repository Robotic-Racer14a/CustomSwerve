package frc.robot.subsystems.superstructue;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    TalonFX shootMotor = new TalonFX(40);

    double SHOOT_ANGLE = Units.degreesToRadians(15);
    double SHOOTER_HEIGHT = Units.inchesToMeters(22);

    DCMotorSim shootMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.001, 1
        ),
        DCMotor.getKrakenX60Foc(1)
        );

    double kp = 0, kv = 0;
    
    public ShooterSubsystem () {
        
        TalonFXConfiguration shootMotorConfig = new TalonFXConfiguration();
        shootMotorConfig.CurrentLimits.StatorCurrentLimit = 90;
        shootMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shootMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        shootMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shootMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shootMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shootMotorConfig.Slot0.kP = kp;
        shootMotorConfig.Slot0.kI = 0;
        shootMotorConfig.Slot0.kD = 0;
        shootMotorConfig.Slot0.kS = 0;
        shootMotorConfig.Slot0.kV = kv;
        shootMotor.getConfigurator().apply(shootMotorConfig);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velo", getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        double newV = 0.12368, newP = 0.175;
        if (kv != newV || kp != newP) {
            kp = newP;
            kv = newV;
            System.out.println("Updated!");
            shootMotor.getConfigurator().apply(new Slot0Configs().withKP(newP).withKV(newV));
        }
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    public void setVelocity(double velo) {
        velo *= 6;
        shootMotor.setControl(new VelocityVoltage(velo));
    }

    public double getVelocity() {
        return shootMotor.getVelocity().getValueAsDouble() / 6.0;
    }
    
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        shootMotor.getSimState().setSupplyVoltage(supplyVoltage);
        shootMotorSim.setInputVoltage(shootMotor.getSimState().getMotorVoltageMeasure().in(Volts));
        shootMotorSim.update(dtSeconds);
        shootMotor.getSimState().setRawRotorPosition(shootMotorSim.getAngularPosition());
        shootMotor.getSimState().setRotorVelocity(shootMotorSim.getAngularVelocity());
    }
}
