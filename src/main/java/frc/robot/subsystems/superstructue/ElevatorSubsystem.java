package frc.robot.subsystems.superstructue;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.States;

public class ElevatorSubsystem extends SubsystemBase{
    TalonFX elevatorMotor = new TalonFX(20);
    ProfiledPIDController elevatorPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(3, 2));
    ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0, 0, 0);

    DCMotorSim elevatorMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.001, 1
        ),
        DCMotor.getKrakenX60Foc(1)
        );

    MechanismLigament2d elevatorMech = new MechanismLigament2d("elevator", 1, 90);





    public ElevatorSubsystem() {
    }





    @Override
    public void periodic() {
        setTarget(getTarget(Robot.targetState));

        if(!isElevatorAtTarget() && !Robot.armClear) setTarget(getCurrent());
        else runToTarget();

        elevatorMech.setLength(getCurrent());
    }

    @Override
    public void simulationPeriodic() {
        elevatorPID.setPID(100, 0, 10);
        elevatorFF.setKs(0);
        elevatorFF.setKg(0);
        elevatorFF.setKv(7);
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }





    public MechanismLigament2d getMechanism() {
        return elevatorMech;
    }


    public double getTarget(States targetState) {
        switch (targetState) {
            case LEVEL_ONE:
                return 1.2;
            case LEVEL_TWO:
                return 2.0;
            default:
                return 0.5;
        }
    }


    public double getCurrent() {
        return elevatorMotor.getPosition().getValueAsDouble() * 0.01;
    }

    public void setTarget(double target) {
        elevatorPID.setGoal(target);
    }

    public void setVoltage(double volts) {
        volts += elevatorFF.calculate(elevatorPID.getSetpoint().velocity);

        elevatorMotor.setVoltage(volts);
    }

    public void runToTarget() {
        double output = elevatorPID.calculate(getCurrent());

        setVoltage(output);
    }


    public boolean isElevatorAtTarget() {
        return Math.abs(getCurrent() - getTarget(Robot.targetState)) < .5;
    }



    public void updateSimState(double dtSeconds, double supplyVoltage) {
        elevatorMotor.getSimState().setSupplyVoltage(supplyVoltage);
        elevatorMotorSim.setInputVoltage(elevatorMotor.getSimState().getMotorVoltageMeasure().in(Volts));
        elevatorMotorSim.update(dtSeconds);
        elevatorMotor.getSimState().setRawRotorPosition(elevatorMotorSim.getAngularPosition());
        elevatorMotor.getSimState().setRotorVelocity(elevatorMotorSim.getAngularVelocity());
    }
}
