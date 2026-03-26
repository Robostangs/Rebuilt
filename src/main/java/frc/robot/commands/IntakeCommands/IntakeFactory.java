package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.Undex;
import frc.robot.commands.HopperCommands.spitHopper;

public class IntakeFactory {

    public static Command TrueSpit() {
        return new Undex().alongWith(new OutTake()).alongWith(new spitHopper());
    }

    public static Command ballStuck() {
        return new Undex().alongWith(new StartIntake()).alongWith(new spitHopper());

    }

    public static Command OscillateIntake(){
        return new SetPivotPosition(Constants.KirbyConstants.IntakePivotOscillateUpPos).withTimeout(3)
            .andThen(new SetPivotPosition(0.0).withTimeout(3))
                .repeatedly();
    }


}
