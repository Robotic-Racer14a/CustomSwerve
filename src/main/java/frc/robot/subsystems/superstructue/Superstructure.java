package frc.robot.subsystems.superstructue;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase{

    public static enum States {
        LEVEL_TWO,
        LEVEL_THREE,
        PICKUP,
        INTERMEDIATE,
        STOW
    }
    
    ArmSubsystem arm = new ArmSubsystem();
    ElevatorSubsystem elevator = new ElevatorSubsystem();

    States tempState = States.STOW;
    States targetState = States.STOW;
    States previousState = States.STOW;

    public Superstructure() {

        var mech = new Mechanism2d(2, 3);
        var mechRoot = mech.getRoot("Elevator", 1, 0);
        mechRoot.append(elevator.getMechanism()).append(arm.getMechanism());
        SmartDashboard.putData("Mech2d", mech);
    }

    @Override
    public void periodic() {

        if ((targetState == States.LEVEL_THREE || targetState == States.LEVEL_TWO) && previousState != targetState && previousState != States.INTERMEDIATE) {
            tempState = targetState;
            targetState = States.INTERMEDIATE;
        } else if (previousState == States.INTERMEDIATE && targetState == States.INTERMEDIATE) {
            targetState = tempState;
        }

        switch (targetState) {
            case LEVEL_THREE:
                elevator.setTarget(2);
                arm.setTarget(45);
                break;
            case LEVEL_TWO:
                elevator.setTarget(1);
                arm.setTarget(45);
                break;
            case PICKUP:
                elevator.setTarget(0.7);
                arm.setTarget(-45);
                break;
            case INTERMEDIATE:
                elevator.setTarget(elevator.getCurrent());
                arm.setTarget(0);
                break;
            default:
                elevator.setTarget(0.4);
                arm.setTarget(0);
                break;
        }

        if (elevator.isElevatorAtTarget() && arm.isArmAtTarget()) {
            previousState = targetState;
        }

        SmartDashboard.putString("Target State", targetState.toString());
        SmartDashboard.putString("Previous State", previousState.toString());

        if (!elevator.isElevatorAtTarget()) {
            arm.setTarget(0);
            if (!arm.isArmAtTarget()) {
                elevator.setTarget(elevator.getCurrent());
            }
        } 
        elevator.runToTarget();
        arm.runToTarget();
    }

    public void setTargetState(States newTarget) {
        targetState = newTarget;
    }
}
