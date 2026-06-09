// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.superstructue.ArmSubsystem;
import frc.robot.subsystems.superstructue.ElevatorSubsystem;

public class Robot extends TimedRobot {

  public static enum States {
    STOW,
    LEVEL_ONE,
    LEVEL_TWO
  }

  public static States targetState = States.STOW;


  final DriveSubsystem drive = new DriveSubsystem();
  ElevatorSubsystem elevator = new ElevatorSubsystem();
  ArmSubsystem arm = new ArmSubsystem();
  final CommandXboxController driveController = new CommandXboxController(0);

  public static boolean armClear = false;
  public static boolean elevatorAtTarget = false;

  public Robot() {
    var mech = new Mechanism2d(2, 3);
    var mechRoot = mech.getRoot("Elevator", 1, 0);
    mechRoot.append(elevator.getMechanism()).append(arm.getMechanism());

    
    var statics = new Mechanism2d(2, 3);
    statics.getRoot("Branch1", 0, 0.75).append(new MechanismLigament2d("Branch", 0.7, 45));
    statics.getRoot("Branch2", 0, 1.5).append(new MechanismLigament2d("Branch", 0.7, 45));
    SmartDashboard.putData("Mech2d", mech);
    SmartDashboard.putData("Statics", statics);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    armClear = arm.isArmClearOfBranch();
    elevatorAtTarget = elevator.isElevatorAtTarget();
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
      targetState = States.LEVEL_ONE;
    } else if (driveController.b().getAsBoolean()) {
      targetState = States.LEVEL_TWO;
    } else if (driveController.x().getAsBoolean()) {
      targetState = States.STOW;
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
