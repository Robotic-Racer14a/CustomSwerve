package frc.robot.subsystems.superstructue;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    TalonFX turretMotor = new TalonFX(50);

    DCMotorSim turretMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.001, 1
        ),
        DCMotor.getKrakenX60Foc(1)
        );

    ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(999999, 100));
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);
    double target = 0;
    
    public TurretSubsystem () {
        
        TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
        turretMotorConfig.CurrentLimits.StatorCurrentLimit = 90;
        turretMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        turretMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turretMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turretMotor.getConfigurator().apply(turretMotorConfig);
        // pid.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Target", target);
        SmartDashboard.putNumber("Turret/Current", getCurrent());
        runToTarget();
    }

    @Override
    public void simulationPeriodic() {
        pid.setPID(20, 0, 3.5);
        ff.setKs(0);
        ff.setKv(0.2);
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    public double getCurrent() {
        return turretMotor.getPosition().getValueAsDouble() * 0.1;
    }

    public void setTarget(double target, Rotation2d robotAngle) {
        target -= robotAngle.getRadians();
        if (target > Math.PI) target -= 2 * Math.PI;
        else if (target < -Math.PI) target += 2 * Math.PI;
        this.target = target;
        pid.setGoal(target);
    }

    public void setVoltage(double volts) {
        volts += ff.calculate(pid.getSetpoint().velocity);

        turretMotor.setVoltage(volts);
    }

    public void runToTarget() {
        double output = pid.calculate(getCurrent());

        setVoltage(output);
    }

    
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        turretMotor.getSimState().setSupplyVoltage(supplyVoltage);
        turretMotorSim.setInputVoltage(turretMotor.getSimState().getMotorVoltageMeasure().in(Volts));
        turretMotorSim.update(dtSeconds);
        turretMotor.getSimState().setRawRotorPosition(turretMotorSim.getAngularPosition());
        turretMotor.getSimState().setRotorVelocity(turretMotorSim.getAngularVelocity());
    }
}
