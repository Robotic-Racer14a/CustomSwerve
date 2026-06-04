// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final TalonFX elevatorMotor = new TalonFX(20);
  private final DCMotorSim elevatorMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.001, 10
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    
  private final TalonFX wristMotor = new TalonFX(21);
  private final DCMotorSim wristMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.001, 20
        ),
        DCMotor.getKrakenX60Foc(1)
    );
  private final MechanismLigament2d m_elevator;
  private final MechanismLigament2d m_wrist;

  public Robot() {
    m_robotContainer = new RobotContainer(this::getPeriod);

    // the main mechanism object
    Mechanism2d mech = new Mechanism2d(3, 3);
    // the mechanism root node
    MechanismRoot2d root = mech.getRoot("climber", 2, 0);

    // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // off the root node or another ligament object
    m_elevator = root.append(new MechanismLigament2d("elevator", 0, 90));
    m_wrist =
        m_elevator.append(
            new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));

    // post the mechanism to the dashboard
    SmartDashboard.putData("Mech2d", mech);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    elevatorMotor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    elevatorMotorSim.setInputVoltage(elevatorMotor.getSimState().getMotorVoltageMeasure().in(Volts));
    elevatorMotorSim.update(0.02);
    elevatorMotor.getSimState().setRawRotorPosition(elevatorMotorSim.getAngularPosition().in(Rotations) * 10);
    elevatorMotor.getSimState().setRotorVelocity(elevatorMotorSim.getAngularVelocity().in(RotationsPerSecond) * 10);

    wristMotor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    wristMotorSim.setInputVoltage(wristMotor.getSimState().getMotorVoltageMeasure().in(Volts));
    wristMotorSim.update(0.02);
    wristMotor.getSimState().setRawRotorPosition(wristMotorSim.getAngularPosition().in(Rotations) * 20);
    wristMotor.getSimState().setRotorVelocity(wristMotorSim.getAngularVelocity().in(RotationsPerSecond) * 20);

    m_elevator.setLength(elevatorMotor.getPosition().getValueAsDouble() / 10);
    m_wrist.setAngle((wristMotor.getPosition().getValueAsDouble() / 20) * 360);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  boolean runToPosition = false;
  SwerveModuleState target = new SwerveModuleState(MetersPerSecond.of(0), Rotation2d.k180deg);


  @Override
  public void teleopPeriodic() {
    if (m_robotContainer.driveController.a().getAsBoolean()) {
      wristMotor.set(0.2);
      runToPosition = true;
    }else if (m_robotContainer.driveController.b().getAsBoolean()) {
      wristMotor.set(-0.2);
    } else {
      wristMotor.set(0);
      runToPosition = false;
    }

    if (m_robotContainer.driveController.x().getAsBoolean()) {
      elevatorMotor.set(0.2);
      target =  new SwerveModuleState(MetersPerSecond.of(1), Rotation2d.kZero);
    }else if (m_robotContainer.driveController.y().getAsBoolean()) {
      elevatorMotor.set(-0.2);
      target =  new SwerveModuleState(MetersPerSecond.of(0), Rotation2d.k180deg);
    } else {
      elevatorMotor.set(0);
    }

    // if (runToPosition) {
      //m_robotContainer.drive.setSwerveStates(target);
    // }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

public void simulationInit() {
   
}

@Override
public void simulationPeriodic() {
  
  m_robotContainer.drive.updateSimState(0.02, RobotController.getBatteryVoltage());
}
}
