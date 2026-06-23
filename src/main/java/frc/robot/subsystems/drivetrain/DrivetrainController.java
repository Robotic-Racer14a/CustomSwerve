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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainController extends SubsystemBase{

    DriveSubsystem drive = new DriveSubsystem();

    private final PIDController translationalController = new PIDController(5, 0, 0.4);
    private final SlewRateLimiter accelerationLimiter = new SlewRateLimiter(100, -4, 0); 

    private double previousDriveToPoseTime;
    private double previousDriveToPoseDirection;

    private Pose2d targetPose = new Pose2d(1,1, Rotation2d.kZero); //Pose2d.kZero;
    private LinearVelocity maxPIDSpeed = MetersPerSecond.of(3), defaultPIDSpeed = maxPIDSpeed;
    private AngularVelocity maxPIDAngularSpeed = RotationsPerSecond.of(1);
    private double distanceUntilDone = 0.25, defaultDistance = distanceUntilDone;

    public DrivetrainController() {

    }

    @Override
    public void periodic() {
        translationalController.setPID(5, 0, 0.4);
        // targetPose = new Pose2d(0,0, Rotation2d.kZero);
        targetPose = new Pose2d(5,5, Rotation2d.kZero);
        driveToPosition();
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

        
        SmartDashboard.putNumber("Actual Commanded Speed", Math.sqrt(
            Math.pow(translationalOutput * Math.cos(limitedAngleToPose), 2) + 
            Math.pow(translationalOutput * Math.sin(limitedAngleToPose), 2)
            ));
        
        drive.drive(translationalOutput * Math.cos(limitedAngleToPose), translationalOutput * Math.sin(limitedAngleToPose), 0, 0.02);
                        
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
