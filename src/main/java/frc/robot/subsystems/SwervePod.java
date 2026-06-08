package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwervePod extends SubsystemBase{

    private static final double kDriveRotationsPerMeter = 10 / (Units.inchesToMeters(2) * 2 * Math.PI);
  
    
    private static final double kModuleMaxAngularVelocity = DriveSubsystem.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration =
        4 * 2 * Math.PI; // radians per second squared
    
    private final TalonFX driveMotor, turnMotor;
    private final CANcoder canCoder;

    private final DCMotorSim driveMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.001, 1
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    private final DCMotorSim turnMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.001, 1
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    private final SimpleMotorFeedforward turningFF = new SimpleMotorFeedforward(0, 0.04);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController =
        new ProfiledPIDController(
            8,
            0,
            1,
            new TrapezoidProfile.Constraints(300, 300));
                //  kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));
    
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
        driveMotorConfig.Slot0.kP = 0.25;
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

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    ////////////////////////////// Getters ///////////////////////////////
    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        if (Utils.isSimulation()) {
            return new SwerveModuleState(
                driveMotorSim.getAngularVelocity().in(RevolutionsPerSecond) / kDriveRotationsPerMeter, Rotation2d.fromRotations(turnMotorSim.getAngularPositionRotations() / 19));
        }
        return new SwerveModuleState(
            driveMotor.getVelocity().getValueAsDouble() / kDriveRotationsPerMeter, new Rotation2d(canCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        if (Utils.isSimulation()) {
            return new SwerveModulePosition(
                driveMotorSim.getAngularPosition().in(Rotations) / kDriveRotationsPerMeter, Rotation2d.fromRotations(turnMotorSim.getAngularPositionRotations() / 19));    
        }
        return new SwerveModulePosition(
            driveMotor.getPosition().getValueAsDouble() * kDriveRotationsPerMeter, new Rotation2d(canCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));
    }

    public void updateSimState(double dtSeconds, double supplyVoltage) {
        driveMotor.getSimState().setSupplyVoltage(supplyVoltage);
        turnMotor.getSimState().setSupplyVoltage(supplyVoltage);
        driveMotorSim.setInputVoltage(driveMotor.getSimState().getMotorVoltageMeasure().in(Volts));
        turnMotorSim.setInputVoltage(turnMotor.getSimState().getMotorVoltageMeasure().in(Volts));
        driveMotorSim.update(dtSeconds);
        turnMotorSim.update(dtSeconds);
        driveMotor.getSimState().setRawRotorPosition(driveMotorSim.getAngularPosition());
        driveMotor.getSimState().setRotorVelocity(driveMotorSim.getAngularVelocity());
        turnMotor.getSimState().setRawRotorPosition(turnMotorSim.getAngularPosition());
        turnMotor.getSimState().setRotorVelocity(turnMotorSim.getAngularVelocity());
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

        setState(desiredState);
    }

    public void setState(SwerveModuleState desiredState) {
        // Calculate the turning motor output from the turning PID controller.
        m_turningPIDController.setPID(4, 0, 0.8);
        final double turnOutput =
            m_turningPIDController.calculate(
                getPosition().angle.getRadians(), desiredState.angle.getRadians());

        turningFF.setKv(0.3);
        final double turnFeedforward =
            turningFF.calculate(m_turningPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber("Target Turn Velo", m_turningPIDController.getSetpoint().velocity);
        driveMotor.setControl(new VelocityVoltage(desiredState.speedMetersPerSecond * kDriveRotationsPerMeter));
        turnMotor.setVoltage(turnOutput + turnFeedforward);
    }
}
