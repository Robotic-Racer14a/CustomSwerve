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
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    double target = 0;




    public ElevatorSubsystem() {
        elevatorPID.setTolerance(0.3, 2);
    }





    @Override
    public void periodic() {
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


    public double getCurrent() {
        return elevatorMotor.getPosition().getValueAsDouble() * 0.01;
    }

    public void setTarget(double target) {
        this.target = target;
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
        return Math.abs(target - getCurrent()) < 0.2;
    }

    public void updateSimState(double dtSeconds, double supplyVoltage) {
        elevatorMotor.getSimState().setSupplyVoltage(supplyVoltage);
        elevatorMotorSim.setInputVoltage(elevatorMotor.getSimState().getMotorVoltageMeasure().in(Volts));
        elevatorMotorSim.update(dtSeconds);
        elevatorMotor.getSimState().setRawRotorPosition(elevatorMotorSim.getAngularPosition());
        elevatorMotor.getSimState().setRotorVelocity(elevatorMotorSim.getAngularVelocity());
    }
}
