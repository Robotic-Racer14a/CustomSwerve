package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

public class FieldCentricCommand extends Command{
    private final DriveSubsystem drive;
    private final CommandXboxController driveController;
    private final Supplier<Double> time;

    public FieldCentricCommand(DriveSubsystem drive, CommandXboxController driveController, Supplier<Double> time) {
        this.drive = drive;
        this.driveController = driveController;
        this.time = time;
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
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

        drive.drive(xSpeed, ySpeed, rot, time.get());
    }

    @Override
    public void end(boolean interupted) {
        drive.drive(0, 0, 0, 0);
    }
}
