package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FeederCommands.Undex;
import frc.robot.commands.HopperCommands.spitHopper;

public class IntakeFactory {

    public static Command TrueSpit() {
        return new Undex().alongWith(new OutTake()).alongWith(new spitHopper());
    }

    public static Command ballStuck() {
        return new Undex().alongWith(new StartIntake(() -> -1)).alongWith(new spitHopper());

    }

    public static Command OscillateIntake(){
        return (
            new SetPivotPosition(0.145)
            .andThen(new WaitCommand(0.25))
            .andThen(new SetPivotPosition(Constants.KirbyConstants.kExtendSetpoint))
            .andThen(new WaitCommand(0.25)))
            .repeatedly();
    }


}
