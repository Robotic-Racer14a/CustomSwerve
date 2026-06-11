package frc.robot.subsystems.superstructue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    public static enum States {
        HIGH,
        MID,
        PICKUP,
        INTERMEDIATE,
        STOW
    }

    double ARM_MOUNT_Y = Units.inchesToMeters(27);
    double ARM_MOUNT_X = Units.inchesToMeters(11);
    boolean enableIntermediateStow = false;
    
    ArmSubsystem bottomArm = new ArmSubsystem();
    TopArmSubsystem topArm = new TopArmSubsystem();
    States targetState = States.STOW;
    States tempTarget = States.STOW;
    States previousState = States.STOW;

    public Superstructure() {
        var mech = new Mechanism2d(2, 3);
        var mechRoot = mech.getRoot("Arm", 1 + ARM_MOUNT_X, 0);
        mechRoot.append(new MechanismLigament2d("Mount", ARM_MOUNT_Y, 90)).append(bottomArm.getMechanism()).append(topArm.getMechanism());
        
        SmartDashboard.putData("Mech2d", mech);

    }

    @Override
    public void periodic() {

        if ((previousState == States.MID && targetState == States.HIGH) || (previousState == States.HIGH && targetState == States.MID)) {
            tempTarget = targetState;
            targetState = States.INTERMEDIATE;
        } else if (targetState == States.INTERMEDIATE && previousState == States.INTERMEDIATE) {
            targetState = tempTarget;
        }

        switch (targetState) {
            case HIGH:
                enableIntermediateStow = true;
                setTargetAngles(Units.inchesToMeters(57), Units.inchesToMeters(46)); 
                break;
            case MID:
                enableIntermediateStow = true;
                setTargetAngles(Units.inchesToMeters(41), Units.inchesToMeters(34)); 
                break;
            case INTERMEDIATE:
                enableIntermediateStow = true;
                setTargetAngles(Units.inchesToMeters(37), Units.inchesToMeters(52));
                break;
            case PICKUP:
                enableIntermediateStow = true;
                setTargetAngles(Units.inchesToMeters(-30), Units.inchesToMeters(40));
                break;
            default:
                if (enableIntermediateStow) {
                    boolean moveBottom = true;
                    topArm.setTarget(topArm.getCurrent());
                    bottomArm.setTarget(90);

                    if (topArm.getCurrent() < -90) {
                        moveBottom = false;
                        topArm.setTarget(-90);
                        bottomArm.setTarget(bottomArm.getCurrent());
                    }

                    if ((Math.abs(bottomArm.getCurrent() - 90) < 5 && moveBottom) || ((Math.abs(-90 - topArm.getCurrent()) < 5) && !moveBottom)) enableIntermediateStow = false;
                } else {
                    previousState = States.STOW;
                    topArm.setTarget(-150);
                    bottomArm.setTarget(210);
                }
                break;
        }
        topArm.runToTarget();
        bottomArm.runToTarget();
    }

    public void setTargetState(States newTarget) {
        targetState = newTarget;
    }

    public void setTargetAngles(double x, double y) {
        y -= ARM_MOUNT_Y;
        x -= ARM_MOUNT_X;
        double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        double baseAngle = Math.acos((Math.pow(topArm.TOP_ARM_LENGTH, 2) + Math.pow(distance, 2) - Math.pow(bottomArm.BOTTOM_ARM_LENGTH, 2)) / (2.0 * topArm.TOP_ARM_LENGTH * distance));
        double topAngle = Math.acos((Math.pow(bottomArm.BOTTOM_ARM_LENGTH, 2) + Math.pow(topArm.TOP_ARM_LENGTH, 2) - Math.pow(distance, 2)) / (2.0 * bottomArm.BOTTOM_ARM_LENGTH * topArm.TOP_ARM_LENGTH));

        double offsetAngle = getOffsetAngle(x, y, baseAngle, topAngle);

        if (Units.radiansToDegrees(offsetAngle) > 90) {baseAngle = offsetAngle - baseAngle; topAngle = (-topAngle + Math.PI);}
        else {baseAngle += offsetAngle; topAngle = -(Math.PI - topAngle);}

        if (Math.abs(topArm.getCurrent() - Units.radiansToDegrees(topAngle)) < 5 && Math.abs(bottomArm.getCurrent() - Units.radiansToDegrees(baseAngle)) < 5) previousState = targetState;
        topArm.setTarget(Units.radiansToDegrees(topAngle));
        bottomArm.setTarget(Units.radiansToDegrees(baseAngle));
    }

    public double getOffsetAngle(double targetX, double targetY, double baseAngle, double topAngle) {
        double currentX = 0;
        double currentY = 0;
        currentX += bottomArm.BOTTOM_ARM_LENGTH * Math.cos(baseAngle);
        currentY += bottomArm.BOTTOM_ARM_LENGTH * Math.sin(baseAngle);

        double newTopAngle = topAngle - ((Math.PI/2.0) - baseAngle);
        currentX += topArm.TOP_ARM_LENGTH * Math.sin(newTopAngle);
        currentY -= topArm.TOP_ARM_LENGTH * Math.cos(newTopAngle);

        return Math.atan2(targetY, targetX) - Math.atan2(currentY, currentX);
    }

    public ArmSubsystem getBottomArm() {
        return bottomArm;
    }

    public TopArmSubsystem getTopArm() {
        return topArm;
    }
}
