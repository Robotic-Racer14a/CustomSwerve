// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainController;
import frc.robot.subsystems.superstructue.Superstructure;

public class Robot extends TimedRobot {

  DrivetrainController drive = new DrivetrainController();
  Superstructure superstructure = new Superstructure();
  final CommandXboxController driveController = new CommandXboxController(0);

  public static boolean armClear = false;
  public static boolean elevatorAtTarget = false;

  public Robot() {

    
    var statics = new Mechanism2d(2, 3);
    statics.getRoot("Branch1", 0, Units.inchesToMeters(21.875) - 0.4).append(new MechanismLigament2d("Branch", 0.7, 35));
    statics.getRoot("Branch2", 0, Units.inchesToMeters(47.625) - 0.4).append(new MechanismLigament2d("Branch", 0.7, 35));
    SmartDashboard.putData("Statics", statics);
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
      superstructure.setTargetState(Superstructure.States.HIGH);
    } else if (driveController.b().getAsBoolean()) {
      superstructure.setTargetState(Superstructure.States.MID);
    } else if (driveController.x().getAsBoolean()) {
      superstructure.setTargetState(Superstructure.States.LOW);
    } else if (driveController.y().getAsBoolean()) {
      superstructure.setTargetState(Superstructure.States.STOW);
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
