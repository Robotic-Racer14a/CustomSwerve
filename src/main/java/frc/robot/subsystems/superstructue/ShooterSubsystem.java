package frc.robot.subsystems.superstructue;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    TalonFX shootMotor = new TalonFX(40);

    DCMotorSim shootMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.001, 1
        ),
        DCMotor.getKrakenX60Foc(1)
        );
    
    public ShooterSubsystem () {

    }





    
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        shootMotor.getSimState().setSupplyVoltage(supplyVoltage);
        shootMotorSim.setInputVoltage(shootMotor.getSimState().getMotorVoltageMeasure().in(Volts));
        shootMotorSim.update(dtSeconds);
        shootMotor.getSimState().setRawRotorPosition(shootMotorSim.getAngularPosition());
        shootMotor.getSimState().setRotorVelocity(shootMotorSim.getAngularVelocity());
    }
}
