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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
  
  ElevatorSim sim = new ElevatorSim(DCMotor.getKrakenX60(2), 10, 1, Units.inchesToMeters(2), 0.5, 3, true, 0.5);
  SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 10, 0.2, Units.inchesToMeters(10), Units.degreesToRadians(-110), Units.degreesToRadians(280), true, 0);

  // the main mechanism object
    Mechanism2d mech = new Mechanism2d(3, 5);
    // the mechanism root node
    MechanismRoot2d root = mech.getRoot("climber", 1.5, 0);
  
  MechanismLigament2d m_elevatorMech2d =
      root.append(
          new MechanismLigament2d("Elevator", sim.getPositionMeters(), 90));

  MechanismLigament2d armMech2d =
      m_elevatorMech2d.append(
          new MechanismLigament2d("Arm", Units.inchesToMeters(18), Units.radiansToDegrees(armSim.getAngleRads()) - 90, 10,  new Color8Bit(0, 0, 190)));

  public Robot() {
    m_robotContainer = new RobotContainer(this::getPeriod);



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

    // m_elevator.setLength(elevatorMotor.getPosition().getValueAsDouble() / 10);
    // m_wrist.setAngle((wristMotor.getPosition().getValueAsDouble() / 20) * 360);
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


  ProfiledPIDController elevatorPID = new ProfiledPIDController(400, 0, 0, new TrapezoidProfile.Constraints(3, 3));
  ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0, 0.02, 6.3);

  ProfiledPIDController armPID = new ProfiledPIDController(5, 0, 1, new TrapezoidProfile.Constraints(Units.degreesToRadians(360), Units.degreesToRadians(360)));
  ArmFeedforward armFF = new ArmFeedforward(0, 1.98, 0.263);

  @Override
  public void teleopPeriodic() {
    if (m_robotContainer.driveController.a().getAsBoolean()) elevatorPID.setGoal(1);
    else elevatorPID.setGoal(2);
    double output = elevatorPID.calculate(sim.getPositionMeters());
    output += elevatorFF.calculate(elevatorPID.getSetpoint().velocity);
    
    sim.setInputVoltage(output);
    sim.update(0.02);
    m_elevatorMech2d.setLength(sim.getPositionMeters());

    if (m_robotContainer.driveController.b().getAsBoolean()) armPID.setGoal(Units.degreesToRadians(45));
    else if (m_robotContainer.driveController.x().getAsBoolean()) armPID.setGoal(Units.degreesToRadians(180));
    else armPID.setGoal(Units.degreesToRadians(90));
    double armOutput = armPID.calculate(armSim.getAngleRads());
    armOutput += armFF.calculate(armPID.getSetpoint().position, armPID.getSetpoint().velocity);

    armSim.setInputVoltage(armOutput);
    armSim.update(0.02);
    armMech2d.setAngle(Units.radiansToDegrees(armSim.getAngleRads()) - 90);
    SmartDashboard.putNumber("Arm Error", Units.radiansToDegrees(armPID.getPositionError()));
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
