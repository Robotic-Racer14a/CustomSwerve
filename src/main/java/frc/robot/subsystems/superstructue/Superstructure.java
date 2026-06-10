package frc.robot.subsystems.superstructue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    public static enum States {
        HIGH,
        MID,
        PICKUP,
        STOW
    }

    double TOP_ARM_LENGTH = Units.inchesToMeters(25);
    double BOTTOM_ARM_LENGTH = Units.inchesToMeters(25);
    
    ArmSubsystem bottomArm = new ArmSubsystem();
    TopArmSubsystem topArm = new TopArmSubsystem();
    States targetState = States.STOW;

    public Superstructure() {
        var mech = new Mechanism2d(2, 3);
        var mechRoot = mech.getRoot("Arm", 1, 0);
        mechRoot.append(bottomArm.getMechanism()).append(topArm.getMechanism());

        
        SmartDashboard.putData("Mech2d", mech);

    }

    @Override
    public void periodic() {
        switch (targetState) {
            case HIGH:
                setTargetAngles(Units.inchesToMeters(32), Units.inchesToMeters(12));
                break;
            case MID:
                setTargetAngles(Units.inchesToMeters(0), Units.inchesToMeters(45));
                break;
            case PICKUP:
                setTargetAngles(Units.inchesToMeters(-32), Units.inchesToMeters(30));
                break;
            default:
                topArm.setTarget(-150);
                bottomArm.setTarget(160);
                break;
        }
    }

    public void setTargetState(States newTarget) {
        targetState = newTarget;
    }

    public void setTargetAngles(double x, double y) {
        double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        double offsetAngle = Math.atan2(y, x);

        double baseAngle = Math.acos((Math.pow(TOP_ARM_LENGTH, 2) + Math.pow(distance, 2) - Math.pow(BOTTOM_ARM_LENGTH, 2)) / (2 * TOP_ARM_LENGTH * distance)) + offsetAngle;
        double topAngle = Math.acos((Math.pow(BOTTOM_ARM_LENGTH, 2) + Math.pow(TOP_ARM_LENGTH, 2) - Math.pow(distance, 2)) / (2 * BOTTOM_ARM_LENGTH * TOP_ARM_LENGTH));

        topArm.setTarget(-(180 - Units.radiansToDegrees(topAngle)));
        bottomArm.setTarget(Units.radiansToDegrees(baseAngle));
    }

    public ArmSubsystem getBottomArm() {
        return bottomArm;
    }

    public TopArmSubsystem getTopArm() {
        return topArm;
    }
}
