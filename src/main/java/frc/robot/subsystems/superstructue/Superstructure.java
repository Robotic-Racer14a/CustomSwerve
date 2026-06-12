package frc.robot.subsystems.superstructue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase{

    public static enum States {
        LEVEL_TWO,
        LEVEL_THREE,
        PICKUP,
        INTERMEDIATE,
        STOW
    }

    double MOUNT_X = Units.inchesToMeters(-10), MOUNT_Y = Units.inchesToMeters(15);
    
    ArmSubsystem arm = new ArmSubsystem();
    ElevatorSubsystem elevator = new ElevatorSubsystem();

    MechanismLigament2d mechLig = new MechanismLigament2d("arm", Units.inchesToMeters(18), 0, 10, new Color8Bit(Color.kBlue));

    States tempState = States.STOW;
    States targetState = States.STOW;
    States previousState = States.STOW;

    public Superstructure() {

        var mech = new Mechanism2d(4, 3);
        var mechRoot = mech.getRoot("Elevator", 2 + MOUNT_X, MOUNT_Y);
        mechRoot.append(mechLig);
        SmartDashboard.putData("Mech2d", mech);
    }

    @Override
    public void periodic() {
        mechLig.setLength(elevator.getCurrent());
        mechLig.setAngle(arm.getCurrent());

        if ((targetState == States.LEVEL_THREE || targetState == States.LEVEL_TWO) && previousState != targetState && previousState != States.INTERMEDIATE) {
            tempState = targetState;
            targetState = States.INTERMEDIATE;
        } else if (previousState == States.INTERMEDIATE && targetState == States.INTERMEDIATE) {
            targetState = tempState;
        }

        switch (targetState) {
            case LEVEL_THREE:
                setTargets(Units.inchesToMeters(57), Units.inchesToMeters(46));
                break;
            case LEVEL_TWO:
                setTargets(Units.inchesToMeters(41), Units.inchesToMeters(34));
                break;
            case PICKUP:
                setTargets(Units.inchesToMeters(-40), Units.inchesToMeters(40));
                break;
            case INTERMEDIATE:
                setTargets(Units.inchesToMeters(41), Units.inchesToMeters(46));
                break;
            default:
                setTargets(Units.inchesToMeters(10), Units.inchesToMeters(5));
                break;
        }

        if (elevator.isElevatorAtTarget() && arm.isArmAtTarget()) {
            previousState = targetState;
        }

        SmartDashboard.putString("Target State", targetState.toString());
        SmartDashboard.putString("Previous State", previousState.toString());

        if (!arm.isArmAtTarget(20) && arm.getCurrent() > 0) {
            elevator.setTarget(elevator.getCurrent() > 1.0 ? 1.0 : elevator.getCurrent());
            if (!elevator.isElevatorAtTarget()) {
                arm.setTarget(arm.getCurrent());
            }
        }

        elevator.runToTarget();
        arm.runToTarget(elevator.getCurrent());
    }

    public void setTargetState(States newTarget) {
        targetState = newTarget;
    }

    public void setTargets(double x, double y) {
        x -= MOUNT_X;
        y -= MOUNT_Y;

        arm.setTarget(Units.radiansToDegrees(Math.atan2(y, x)));
        elevator.setTarget(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
    }
}
