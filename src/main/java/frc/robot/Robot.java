// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer(this::getPeriod);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
      runToPosition = true;
    } else {
      runToPosition = false;
    }

    if (m_robotContainer.driveController.x().getAsBoolean()) {
      target =  new SwerveModuleState(MetersPerSecond.of(1), Rotation2d.kZero);
    }

    if (m_robotContainer.driveController.y().getAsBoolean()) {
      target =  new SwerveModuleState(MetersPerSecond.of(0), Rotation2d.k180deg);
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
