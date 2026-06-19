package frc.robot.subsystems.superstructue;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase{

    Translation2d redGoal = new Translation2d(Units.inchesToMeters(650.12 - 180.56), Units.inchesToMeters(158.32));

    public static enum States {
        SCORING,
        PASSING,
        STOPPED
    }

    public static enum SecondaryStates {
        INTAKE,
        SHOOTING,
        SHOOTANDINTAKE,
        STOPPED
    }
    
    ShooterSubsystem shooter = new ShooterSubsystem();
    TurretSubsystem turret = new TurretSubsystem();
    States targetState = States.STOPPED;
    SecondaryStates targetSecondary = SecondaryStates.STOPPED;
    Supplier<Pose2d> robotPose; 
    Supplier<ChassisSpeeds> speedsSupplier;

    
    
    StructPublisher<Pose2d> firstPassPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Piece Ending", Pose2d.struct).publish();

    StructPublisher<Pose2d> shootTargetPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Shoot Target", Pose2d.struct).publish();

    public Superstructure(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speedsSupplier) {
        this.robotPose = robotPose;
        this.speedsSupplier = speedsSupplier;
    }

    @Override
    public void periodic() {
        
        // shootTargetPublisher.set(new Pose2d(redGoal, Rotation2d.kZero));
        setTargets(redGoal, 0);
        firstPassPublisher.set(new Pose2d(getLandingPosition(turret.getCurrent() + getTurretPose().getRotation().getRadians(), shooter.getVelocity()), Rotation2d.kZero));

        switch (targetState) {
            case SCORING:
                break;
            case PASSING:
                break;
            default:
                break;
        }

        switch (targetSecondary) {
            case INTAKE:
            case SHOOTING:
            case SHOOTANDINTAKE:
            default:
        }

    }

    public void setTargetState(States newTarget) {
        targetState = newTarget;
    }

    public void setSecondaryState(SecondaryStates newTarget) {
        targetSecondary = newTarget;
    }

    public Pose2d getTurretPose() {
        return robotPose.get().plus(new Transform2d(Units.inchesToMeters(10), 0, Rotation2d.kZero));
    }

    public void setTargets(Translation2d shotTarget, double counter) {
        Pose2d turretPose = getTurretPose();
        double distance = Math.sqrt(shotTarget.getSquaredDistance(turretPose.getTranslation()));
        
        SmartDashboard.putNumber("Distance", distance);
        if (distance < 1.25) {
            distance = 1.25;
        } else if (distance > 10) {
            distance = 10;

        }

        double sqrtNum = (distance * distance) * - 10.0;
        double sqrtDenom = (2.0 * (1.8 - shooter.SHOOTER_HEIGHT)) - (2.0 * distance * Math.tan((Math.PI / 2.0) - shooter.SHOOT_ANGLE));

        double vel = Math.sqrt(sqrtNum/sqrtDenom)
                / (Math.cos((Math.PI / 2.0) - shooter.SHOOT_ANGLE));

        double turretAngle = Math.atan2(shotTarget.getY() - turretPose.getY(), shotTarget.getX() - turretPose.getX());
        SmartDashboard.putNumber("Velo", vel);

        shooter.setVelocity(vel);
        turret.setTarget(turretAngle, turretPose.getRotation());
        Translation2d shootingSpot = getLandingPosition(turretAngle, vel);

        if (counter < 1) { //(shootingSpot.getDistance(shotTarget) > 0.5 && !pass) {
            counter ++;
            setTargets(shotTarget.minus(shootingSpot.minus(shotTarget)), counter);
        } else {
            shootTargetPublisher.set(new Pose2d(shotTarget, Rotation2d.kZero));
        }

        
    }

    public Translation2d getLandingPosition(double turretAngle, double shootPower) {
        double verticalVelo = shootPower * Math.cos(shooter.SHOOT_ANGLE);
        double t = verticalVelo + Math.sqrt(Math.pow(verticalVelo, 2) + (20 * (shooter.SHOOTER_HEIGHT - 1.8)));
        t /= 10;

        double horizontalVelo = shootPower * Math.sin(shooter.SHOOT_ANGLE);
        double rotationX = speedsSupplier.get().omegaRadiansPerSecond * Math.cos(getTurretPose().getRotation().getRadians()) * Units.inchesToMeters(10);
        double rotationY = speedsSupplier.get().omegaRadiansPerSecond * Math.sin(getTurretPose().getRotation().getRadians()) * Units.inchesToMeters(10);
        Translation2d shootPose = new Translation2d((horizontalVelo * Math.cos(turretAngle) * t) + speedsSupplier.get().vxMetersPerSecond + rotationX, (horizontalVelo * Math.sin(turretAngle) * t) + speedsSupplier.get().vyMetersPerSecond + rotationY);

        return shootPose.plus(getTurretPose().getTranslation());
    }
}
