// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.FieldCentricCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  private final DriveSubsystem drive = new DriveSubsystem();
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final Supplier<Double> currentTime;

  public RobotContainer(Supplier<Double> currentTime) {
    this.currentTime = currentTime;
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(new FieldCentricCommand(drive, driveController, currentTime));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
