package frc.robot.subsystems.superstructue;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{

    double ARM_WEIGHT = 5;
    double END_EFFECTOR_WEIGHT = 2;
    TalonFX armMotor = new TalonFX(21);
    ProfiledPIDController armPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(200, 200));
    SimpleMotorFeedforward armFF = new SimpleMotorFeedforward(0, 0);
    double kg = 0;

    DCMotorSim armMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.001, 1
        ),
        DCMotor.getKrakenX60Foc(1)
        );

    double targetAngle = 0;



    public ArmSubsystem() {
    }





    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        armPID.setPID(1, 0, 0.2);
        armFF.setKs(0);
        kg = 0;
        armFF.setKv(0.15);
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }





    public double getCurrent() {
        return armMotor.getPosition().getValueAsDouble();
    }

    public void setTarget(double target) {
        targetAngle = target;
        armPID.setGoal(target);
    }

    public void setVoltage(double volts, double extention) {
        volts += armFF.calculate(armPID.getSetpoint().velocity);

        volts += (extention * Math.cos(getCurrent()) + (ARM_WEIGHT / 2) + END_EFFECTOR_WEIGHT) * kg;

        armMotor.setVoltage(volts);
    }

    public void runToTarget(double extention) {
        double output = armPID.calculate(getCurrent());

        setVoltage(output, extention);
    }

    public boolean isArmAtTarget() {
        return isArmAtTarget(5);
    }

    public boolean isArmAtTarget(double tolerance) {
        return Math.abs(getCurrent() - targetAngle) < tolerance;
    }

    public void updateSimState(double dtSeconds, double supplyVoltage) {
        armMotor.getSimState().setSupplyVoltage(supplyVoltage);
        armMotorSim.setInputVoltage(armMotor.getSimState().getMotorVoltageMeasure().in(Volts));
        armMotorSim.update(dtSeconds);
        armMotor.getSimState().setRawRotorPosition(armMotorSim.getAngularPosition());
        armMotor.getSimState().setRotorVelocity(armMotorSim.getAngularVelocity());
    }
}
