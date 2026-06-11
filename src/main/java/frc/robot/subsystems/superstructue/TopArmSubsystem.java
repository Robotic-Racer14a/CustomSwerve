package frc.robot.subsystems.superstructue;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TopArmSubsystem extends SubsystemBase{
    TalonFX armMotor = new TalonFX(21);
    ProfiledPIDController armPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(200, 200));
    ArmFeedforward armFF = new ArmFeedforward(0, 0, 0);
    
    double TOP_ARM_LENGTH = Units.inchesToMeters(25);

    DCMotorSim armMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.001, 1
        ),
        DCMotor.getKrakenX60Foc(1)
        );

    MechanismLigament2d armMech = new MechanismLigament2d("arm", TOP_ARM_LENGTH, -90, 10, new Color8Bit(Color.kRed));





    public TopArmSubsystem() {
    }





    @Override
    public void periodic() {
        armMech.setAngle(getCurrent());
    }

    @Override
    public void simulationPeriodic() {
        armPID.setPID(1, 0, 0.2);
        armFF.setKs(0);
        armFF.setKg(0);
        armFF.setKv(0.15);
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }





    public MechanismLigament2d getMechanism() {
        return armMech;
    }


    public double getCurrent() {
        return armMotor.getPosition().getValueAsDouble();
    }

    public void setTarget(double target) {
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

    

    public void updateSimState(double dtSeconds, double supplyVoltage) {
        armMotor.getSimState().setSupplyVoltage(supplyVoltage);
        armMotorSim.setInputVoltage(armMotor.getSimState().getMotorVoltageMeasure().in(Volts));
        armMotorSim.update(dtSeconds);
        armMotor.getSimState().setRawRotorPosition(armMotorSim.getAngularPosition());
        armMotor.getSimState().setRotorVelocity(armMotorSim.getAngularVelocity());
    }
}
