// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.superstructue.ArmSubsystem;
import frc.robot.subsystems.superstructue.ElevatorSubsystem;

public class Robot extends TimedRobot {


  final DriveSubsystem drive = new DriveSubsystem();
  ElevatorSubsystem elevator = new ElevatorSubsystem();
  ArmSubsystem arm = new ArmSubsystem();
  final CommandXboxController driveController = new CommandXboxController(0);

  public Robot() {
    var mech = new Mechanism2d(3, 3);
    var mechRoot = mech.getRoot("Elevator", 2, 0);
    mechRoot.append(elevator.getMechanism()).append(arm.getMechanism());
    SmartDashboard.putData("Mech2d", mech);
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
    
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    if (driveController.a().getAsBoolean()) {
      elevator.setTarget(200);
    } else {
      elevator.setTarget(100);
    }

    if (driveController.b().getAsBoolean()) {
      arm.setTarget(90);
    } else {
      arm.setTarget(0);
    }
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

  @Override
  public void simulationInit() {
    
  }

  @Override
  public void simulationPeriodic() {
    
    drive.updateSimState(0.02, RobotController.getBatteryVoltage());
  }
}
