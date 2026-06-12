package frc.robot.subsystems.superstructue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase{

    public static enum States {
        SCORING,
        PASSING,
        INTAKE,
        STOW
    }
    

    States tempState = States.STOW;
    States targetState = States.STOW;
    States previousState = States.STOW;

    public Superstructure() {

    }

    @Override
    public void periodic() {


        switch (targetState) {
            case SCORING:
                break;
            case PASSING:
                break;
            case INTAKE:
                break;
            default:
                break;
        }

        SmartDashboard.putString("Target State", targetState.toString());
        SmartDashboard.putString("Previous State", previousState.toString());

    }

    public void setTargetState(States newTarget) {
        targetState = newTarget;
    }
}
