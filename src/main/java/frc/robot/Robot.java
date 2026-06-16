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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.superstructue.Superstructure;
import frc.robot.subsystems.superstructue.Superstructure.States;

public class Robot extends TimedRobot {

  final DriveSubsystem drive = new DriveSubsystem();
  Superstructure superstructure = new Superstructure(drive::getCurrentPose, drive::getCurrentVelocityComponents);
  final CommandXboxController driveController = new CommandXboxController(0);

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
    final var xSpeed =
        -MathUtil.applyDeadband(driveController.getLeftY(), 0.02)
            * DriveSubsystem.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -MathUtil.applyDeadband(driveController.getLeftX(), 0.02)
            * DriveSubsystem.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -MathUtil.applyDeadband(driveController.getRightX(), 0.02)
            * DriveSubsystem.kMaxAngularSpeed;

    drive.drive(xSpeed, ySpeed, rot, this.getPeriod());


    if (driveController.a().getAsBoolean()) {
      superstructure.setTargetState(States.SCORING);
    } else if (driveController.b().getAsBoolean()) {
      superstructure.setTargetState(States.PASSING);
    } else if (driveController.x().getAsBoolean()) {
      superstructure.setTargetState(States.INTAKE);
    } else if (driveController.y().getAsBoolean()) {
      superstructure.setTargetState(States.STOW);
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
