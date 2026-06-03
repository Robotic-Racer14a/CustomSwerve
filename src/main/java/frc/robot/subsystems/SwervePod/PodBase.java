package frc.robot.subsystems.SwervePod;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface PodBase {

    public default SwerveModuleState getState() {
        return new SwerveModuleState();
    }

    public default SwerveModulePosition getPosition() {
        return new SwerveModulePosition();
    }

    public default void setDesiredState(SwerveModuleState desiredState) {}

}
