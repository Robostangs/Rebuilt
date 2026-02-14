package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret.Turret;

public class ClimberFactory {
    public enum climberStates {
        L0, L1, L2, L3, stow
    }

    public static climberStates ClimberState = climberStates.stow;
    static Alert stowAlert = new Alert("Cannot go to stow", Alert.AlertType.kError);
    static Alert pivotAlert = new Alert("Cannot pivot", Alert.AlertType.kError);


    public static Command ClimbToL3() {
        return ClimbToL2()
                .andThen(new SetLevel(Constants.ClimberConstants.kExtendPos))
                .andThen(new SetLevel(Constants.ClimberConstants.kClimbPos))
                .finallyDo(() -> Climber.getInstance().setClimberState(Constants.ClimberConstants.climberStates.L3));

    }

    public static Command ClimbToL2() {
        return ClimbToL1()
                .andThen(new SetLevel(Constants.ClimberConstants.kExtendPos))
                .andThen(new SetLevel(Constants.ClimberConstants.kClimbPos))
                .finallyDo(() -> Climber.getInstance().setClimberState(Constants.ClimberConstants.climberStates.L2));

    }

    public static Command ClimbToL1() {
        return new SetLevel(Constants.ClimberConstants.kExtendPos)
                .andThen(new SetLevel(Constants.ClimberConstants.kClimbPos))
                .finallyDo(() -> Climber.getInstance().setClimberState(Constants.ClimberConstants.climberStates.L1));

    }

    public Command setClimberPivot(double pos) {
        return new InstantCommand(() -> {
            Turret.getInstance().moveTurretSafe();
        })
                .andThen(new WaitUntilCommand(() -> Turret.getInstance().isTurretSafeForClimber()))
                .andThen(new PivotClimber(pos))
                .finallyDo(() -> Turret.setManualTarget(false));
    }

    public static Command deployClimber() {
        return new ClimberFactory().setClimberPivot(Constants.ClimberConstants.kPivotDeployedPos);
    }

    public static Command stowClimber() {
        return new ClimberFactory().setClimberPivot(Constants.ClimberConstants.kPivotDeployedPos);
    }

}
