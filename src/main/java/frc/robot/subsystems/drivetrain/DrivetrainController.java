package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainController extends SubsystemBase{

    public enum DriveStates {
        DRIVE_TO_POINT,
        DRIVER,
        X_MODE
    }

    
    StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Target Pose", Pose2d.struct).publish();

    private final Translation2d chargeStationCorner1 = new Translation2d(2.5,1); //This is measured close to grid on bump side
    private final Translation2d chargeStationCorner2 = new Translation2d(5.3, 4.5); //This is measured away from grid on open side

    DriveSubsystem drive = new DriveSubsystem();

    int rowID = 1; //1 is closest to HP station, 9 is furthest (bump)

    private final PIDController translationalController = new PIDController(5, 0, 0.4);
    private final ProfiledPIDController rotationalController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2));
    private final SlewRateLimiter accelerationLimiter = new SlewRateLimiter(100, -4, 0); 

    private double previousDriveToPoseTime;
    private double previousDriveToPoseDirection;
    private double previousDriveToPoseVelo;

    private Pose2d targetPose = new Pose2d(1,1, Rotation2d.kZero); //Pose2d.kZero;
    private LinearVelocity maxPIDSpeed = MetersPerSecond.of(3), defaultPIDSpeed = maxPIDSpeed;
    private AngularVelocity maxPIDAngularSpeed = RotationsPerSecond.of(1);
    private double distanceUntilDone = 0.25, defaultDistance = distanceUntilDone;

    DriveStates state;

    public DrivetrainController() {
        rotationalController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        translationalController.setPID(5, 0, 0.4);
        rotationalController.setPID(8, 0, 0.4);
        rowID = 8;
        targetPose = new Pose2d(12, 6, Rotation2d.k180deg);
        
        if (state == DriveStates.DRIVE_TO_POINT) {
            targetPose = getTargetPose();
        }
        driveToPosition();
    }

    public void setDriveState (DriveStates state) {
        this.state = state;
    }

    public Pose2d getTargetPose() {
        Pose2d targetPose = Pose2d.kZero;
        switch (rowID) {
            case 1:
                targetPose = new Pose2d(1.85,4.95, Rotation2d.k180deg);
                break;
            case 2:
                targetPose = new Pose2d(1.85,4.42, Rotation2d.k180deg);
                break;
            case 3:
                targetPose = new Pose2d(1.85,3.85, Rotation2d.k180deg);
                break;
            case 4:
                targetPose = new Pose2d(1.85,3.3, Rotation2d.k180deg);
                break;
            case 5:
                targetPose = new Pose2d(1.85,2.77, Rotation2d.k180deg);
                break;
            case 6:    
                targetPose = new Pose2d(1.85,2.2, Rotation2d.k180deg);
                break;
            case 7:
                targetPose = new Pose2d(1.85,1.65, Rotation2d.k180deg);
                break;
            case 8:
                targetPose = new Pose2d(1.85,1.12, Rotation2d.k180deg);
                break;
            case 9:
                targetPose = new Pose2d(1.85,0.55, Rotation2d.k180deg);
                break;
        }

        targetPose = getModifiedTarget(targetPose);

        targetPosePublisher.set(targetPose);
        return targetPose;
    }

    public Pose2d getModifiedTarget(Pose2d targetPose) {
        if (drive.getCurrentPose().getX() < chargeStationCorner1.getX()) {
            if (distanceFromPose(drive.getCurrentPose(), targetPose) > .8) {
                targetPose = targetPose.plus(new Transform2d(-0.5, 0, Rotation2d.kZero));
            }
        } else if (drive.getCurrentPose().getY() > chargeStationCorner2.getY()) {
            if (doesPoseCrossChargeStation(targetPose)) {
                targetPose = new Pose2d(2,4.75, Rotation2d.k180deg);
                if (drive.getCurrentPose().getY() > (chargeStationCorner2.getY() + 0.75)) {
                    targetPose = new Pose2d(5.5, 4.75, Rotation2d.k180deg);
                }
            }
        } else {
            if (doesPoseCrossChargeStation(targetPose)) {
                targetPose = new Pose2d(2,.5, Rotation2d.k180deg);
                if (drive.getCurrentPose().getY() > chargeStationCorner1.getY() && drive.getCurrentPose().getX() > chargeStationCorner2.getX()) {
                    targetPose = new Pose2d(5.5,.5, Rotation2d.k180deg);
                }
            }
        }

        return targetPose;
    }

    public boolean doesPoseCrossChargeStation(Pose2d targetPose) {
        if (drive.getCurrentPose().getX() < chargeStationCorner1.getX()) return false;
        if (Math.abs(targetPose.getX() - drive.getCurrentPose().getX()) < 0.2) return drive.getCurrentPose().getY() < chargeStationCorner1.getY() || drive.getCurrentPose().getY() > chargeStationCorner2.getY();
        
        double slope = (targetPose.getY() - drive.getCurrentPose().getY()) / (targetPose.getX() - drive.getCurrentPose().getX());
        double intersept = targetPose.getY() - (slope * targetPose.getX());

        double crossover1 = (slope * chargeStationCorner1.getX()) + intersept;
        double crossover2 = (slope * chargeStationCorner2.getX()) + intersept;

        if (crossover1 < chargeStationCorner1.getY()) {
            return crossover2 > chargeStationCorner1.getY();
        } else if (crossover1 > chargeStationCorner2.getY()) {
            return crossover2 < chargeStationCorner2.getY();
        }
        return true;
    }

    public void driveToPosition() {

        double distanceAway = distanceFromPose(targetPose, drive.getCurrentPose());

        // Determine the sent velocity of the robot in meters per second
        double translationalOutput = translationalController.calculate(distanceAway);
        translationalOutput = MathUtil.clamp(translationalOutput, -maxPIDSpeed.in(MetersPerSecond),
                maxPIDSpeed.in(MetersPerSecond));
        translationalOutput = accelerationLimiter.calculate(translationalOutput);
        SmartDashboard.putNumber("Commanded Speed", -translationalOutput);
        SmartDashboard.putNumber("Distance Away", distanceAway);

        // Apply velocity in the direction of the anglePose
        double angleToPose = absoluteAngleFromPose(drive.getCurrentPose(), targetPose).getRadians();

        //Limit Heading Change
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - previousDriveToPoseTime;

        double targetChange = angleToPose - previousDriveToPoseDirection;
        SmartDashboard.putNumber("Target Change", targetChange);
        if (targetChange > Math.PI) targetChange -= 2 * Math.PI;
        if (targetChange < -Math.PI) targetChange += 2 * Math.PI;

        //Min value is Math.PI
        double maxDirectionChange = 0;
        if (drive.getCurrentVelocity() > defaultPIDSpeed.in(MetersPerSecond)/2) {
            maxDirectionChange = (1 / drive.getCurrentVelocity()) * (defaultPIDSpeed.in(MetersPerSecond) * Math.PI);
        }

        if (maxDirectionChange == 0) previousDriveToPoseDirection += targetChange;
        else {
            previousDriveToPoseDirection +=
                MathUtil.clamp(
                    targetChange,
                    -maxDirectionChange * elapsedTime,
                    maxDirectionChange * elapsedTime);
        }
        previousDriveToPoseTime = currentTime;
        double limitedAngleToPose = previousDriveToPoseDirection;

        
        if (Rotation2d.fromRadians(limitedAngleToPose).relativeTo(Rotation2d.fromRadians(angleToPose)).getDegrees() > 5 && translationalOutput > 1) {
            translationalOutput = previousDriveToPoseVelo;
        }
        
        previousDriveToPoseVelo = translationalOutput;

        SmartDashboard.putNumber("Actual Commanded Speed", Math.sqrt(
            Math.pow(translationalOutput * Math.cos(limitedAngleToPose), 2) + 
            Math.pow(translationalOutput * Math.sin(limitedAngleToPose), 2)
            ));

        rotationalController.setGoal(targetPose.getRotation().getRadians());
        double rotationalRate = rotationalController.calculate(drive.getCurrentPose().getRotation().getRadians());
        
        drive.drive(translationalOutput * Math.cos(limitedAngleToPose), translationalOutput * Math.sin(limitedAngleToPose), rotationalRate, 0.02);
                        
    }

    public void updateSimState(double dtSeconds, double supplyVoltage) {
        
        drive.updateSimState(dtSeconds, supplyVoltage);
    }


     /////////////////////////// Pose Utility Methods (TODO: Move to new location) //////////////////////////////////////////////////

    /**
     * Method gets the distance of a specified point from another point using a third point as a rotational reference
     * @param measurementPose
     * @param lineOrigin
     * @param lineXPos
     * @return
     */
    public Translation2d translationFromLine(Pose2d measurementPose, Pose2d lineOrigin, Pose2d lineXPos) {
        double lineAngle = Math.atan2(lineXPos.getY() - lineOrigin.getY(), lineXPos.getX() - lineOrigin.getX());
        lineXPos = lineXPos.rotateAround(lineOrigin.getTranslation(), Rotation2d.fromRadians(-lineAngle));
        measurementPose = measurementPose.rotateAround(lineOrigin.getTranslation(), Rotation2d.fromRadians(-lineAngle));

        lineXPos = lineXPos.transformBy(new Transform2d(lineOrigin.getTranslation(),  Rotation2d.kZero));
        measurementPose = measurementPose.transformBy(new Transform2d(lineOrigin.getTranslation(),  Rotation2d.kZero));

        return measurementPose.getTranslation();
    }

    public double distanceFromPose(Pose2d measurementPose, Pose2d origin){
        return Math.sqrt(Math.pow(measurementPose.getX() - origin.getX(), 2) + Math.pow(measurementPose.getY() - origin.getY(), 2));
    }

    public Rotation2d absoluteAngleFromPose(Pose2d measurementPose, Pose2d origin){
        return Rotation2d.fromRadians(Math.atan2(measurementPose.getY() - origin.getY(), measurementPose.getX() - origin.getX()));
    }
}
