package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwervePod.SwervePodReal;

public class DriveSubsystem extends SubsystemBase{

    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private final Translation2d m_frontLeftLocation = new Translation2d(Meter.convertFrom(10.25, Inch), Meter.convertFrom(10.25, Inch));
    private final Translation2d m_frontRightLocation = new Translation2d(Meter.convertFrom(10.25, Inch), -Meter.convertFrom(10.25, Inch));
    private final Translation2d m_backLeftLocation = new Translation2d(-Meter.convertFrom(10.25, Inch), Meter.convertFrom(10.25, Inch));
    private final Translation2d m_backRightLocation = new Translation2d(-Meter.convertFrom(10.25, Inch), -Meter.convertFrom(10.25, Inch));

    private final SwervePodReal m_frontLeft = new SwervePodReal(0, 10, 0, -0.494873046875);
    private final SwervePodReal m_frontRight = new SwervePodReal(1, 11, 1, 0.357421875);
    private final SwervePodReal m_backLeft = new SwervePodReal(2, 12, 2, 0.441650390625);
    private final SwervePodReal m_backRight = new SwervePodReal(3, 13, 3, 0.0647);

    private final Pigeon2 m_gyro = new Pigeon2(0);

    private final SwerveDriveKinematics m_kinematics =
        new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDrivePoseEstimator currentPoseEstimator = 
    new SwerveDrivePoseEstimator(
        m_kinematics, m_gyro.getRotation2d(), 
        new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }, 
            Pose2d.kZero,
            VecBuilder.fill(0.05, 0.05, 0.05),
            VecBuilder.fill(0.5, 0.5, 0.5)
            );
    
    StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Robot Pose", Pose2d.struct).publish();

    public DriveSubsystem() {
        m_gyro.reset();
    }

    @Override
    public void periodic(){
        currentPoseEstimator.update(m_gyro.getRotation2d(), new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            });

        robotPosePublisher.set(getCurrentPose());
    }

    public Pose2d getCurrentPose() {
        m_kinematics.toChassisSpeeds(m_frontLeft.getState(),
                m_frontRight.getState(),
                m_backLeft.getState(),
                m_backRight.getState());
        return currentPoseEstimator.getEstimatedPosition();
    }

    public double getCurrentVelocity() {
        ChassisSpeeds currentSpeed = m_kinematics.toChassisSpeeds(
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        );

        return Math.sqrt(Math.pow(currentSpeed.vxMetersPerSecond, 2) + Math.pow(currentSpeed.vyMetersPerSecond, 2));
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(
        double xSpeed, double ySpeed, double rot, double periodSeconds) {
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, getCurrentPose().getRotation()),
                    periodSeconds));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

}
