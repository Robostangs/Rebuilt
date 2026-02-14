package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.Turret;

public class ClimberFactory {
    public enum climberStates {
        L0, L1, L2, L3, stow
    }

    public static climberStates ClimberState = climberStates.stow;
    static Alert stowAlert = new Alert("Cannot go to stow", Alert.AlertType.kError);
    static Alert pivotAlert = new Alert("Cannot pivot", Alert.AlertType.kError);

    // public static
    // public static Command L0Position(){
    // return new DeferredCommand(() -> {
    // if (
    // }
    // }

    public static Command ClimbToL3() {
        return new SetLevel(Constants.ClimberConstants.kL1Pos)
                .andThen(new SetLevel(Constants.ClimberConstants.kL2Pos)
                        .andThen(new SetLevel(Constants.ClimberConstants.kL3Pos)));

    }

    public static Command ClimbToL2() {
        return new SetLevel(Constants.ClimberConstants.kL1Pos)
                .andThen(new SetLevel(Constants.ClimberConstants.kL2Pos));

    }

    public static Command ClimbToL1() {
        return new SetLevel(Constants.ClimberConstants.kL1Pos);

    }


    public Command setClimberPivot(double pos) {
        // TODO make this smarter
        return new InstantCommand(() -> {
            Turret.setManualTarget(true);
            Turret.getInstance().moveTurretSafe();
        })
            .andThen(new PivotClimber(pos))
            .finallyDo(() -> Turret.setManualTarget(false));
    }
    public static Command deployClimber(){
        return new ClimberFactory().setClimberPivot(Constants.ClimberConstants.kPivotDeployedPos);
    }
    public static Command stowClimber(){
        return new ClimberFactory().setClimberPivot(Constants.ClimberConstants.kPivotDeployedPos);
    }

}
