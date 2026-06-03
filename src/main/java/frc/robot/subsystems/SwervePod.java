package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwervePod extends SubsystemBase{

    private static final double kDriveRotationsPerMeter = Meter.convertFrom(2, Inch) * 0.1; //Wheel Radius * Gear Ratio (here 1 wheel rotation to 10 motor rotations)
  
    private static final double kModuleMaxAngularVelocity = DriveSubsystem.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration =
        2 * Math.PI; // radians per second squared
    
    private final TalonFX driveMotor, turnMotor;
    private final CANcoder canCoder;

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController =
        new ProfiledPIDController(
            30,
            0,
            0,
            new TrapezoidProfile.Constraints(
                 kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0, 0);

    
    public SwervePod(int driveMotorID, int turnMotorID, int encoderID, double offset) {
        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);
        canCoder = new CANcoder(encoderID);

        
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = offset;
        canCoder.getConfigurator().apply(encoderConfig);

        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = 90;
        driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveMotorConfig.Slot0.kP = 0.05;
        driveMotorConfig.Slot0.kI = 0;
        driveMotorConfig.Slot0.kD = 0;
        driveMotorConfig.Slot0.kS = 0;
        driveMotorConfig.Slot0.kV = 0.124;
        driveMotor.getConfigurator().apply(driveMotorConfig);
        
        TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();
        turnMotorConfig.CurrentLimits.StatorCurrentLimit = 50;
        turnMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
        turnMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turnMotor.getConfigurator().apply(turnMotorConfig);
    }

    ////////////////////////////// Getters ///////////////////////////////
    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getVelocity().getValueAsDouble() * kDriveRotationsPerMeter, new Rotation2d(canCoder.getVelocity().getValueAsDouble() * 2 * Math.PI));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getPosition().getValueAsDouble() * kDriveRotationsPerMeter, new Rotation2d(canCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));
    }

    //////////////////////////////////////// Setters ////////////////////////////////////////
    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        var encoderRotation = getPosition().angle;

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(encoderRotation);

        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        desiredState.cosineScale(encoderRotation);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
            m_turningPIDController.calculate(
                getPosition().angle.getRadians(), desiredState.angle.getRadians());

        final double turnFeedforward =
            m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        driveMotor.setControl(new VelocityVoltage(desiredState.speedMetersPerSecond * kDriveRotationsPerMeter));
        turnMotor.setVoltage(turnOutput + turnFeedforward);
    }
}
