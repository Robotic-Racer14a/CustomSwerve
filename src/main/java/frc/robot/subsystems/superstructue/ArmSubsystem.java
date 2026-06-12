package frc.robot.subsystems.superstructue;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    TalonFX armMotor = new TalonFX(21);
    ProfiledPIDController armPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(200, 200));
    ArmFeedforward armFF = new ArmFeedforward(0, 0, 0);

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
        armFF.setKg(0);
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

    public void setVoltage(double volts) {
        volts += armFF.calculate(armPID.getSetpoint().position, armPID.getSetpoint().velocity);

        armMotor.setVoltage(volts);
    }

    public void runToTarget() {
        double output = armPID.calculate(getCurrent());

        setVoltage(output);
    }

    public boolean isArmAtTarget() {
        return Math.abs(getCurrent() - targetAngle) < 5;
    }
    

    public void updateSimState(double dtSeconds, double supplyVoltage) {
        armMotor.getSimState().setSupplyVoltage(supplyVoltage);
        armMotorSim.setInputVoltage(armMotor.getSimState().getMotorVoltageMeasure().in(Volts));
        armMotorSim.update(dtSeconds);
        armMotor.getSimState().setRawRotorPosition(armMotorSim.getAngularPosition());
        armMotor.getSimState().setRotorVelocity(armMotorSim.getAngularVelocity());
    }
}
