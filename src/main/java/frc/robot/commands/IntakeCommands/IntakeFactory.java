package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FeederCommands.Undex;
import frc.robot.commands.HopperCommands.RunHopper;

public class IntakeFactory {

    public static Command TrueSpit() {
        return new Undex().alongWith(new setKirbyWheels(() -> -1)).alongWith(new RunHopper(() -> -0.5));
    }

    public static Command ballStuck() {
        return new Undex().alongWith(new setKirbyWheels(() -> -1)).alongWith(new RunHopper(() -> -0.5));

    }

    public static Command ExtendIntake() {
        return new SetPivotPosition(Constants.KirbyConstants.kExtendSetpoint).alongWith(new setKirbyWheels(() -> 1));
    }

    public static Command RetractIntake() {
        return new SetPivotPosition(Constants.KirbyConstants.kRetractSetpoint);
    }

    public static Command OscillateIntake() {
        return (new SetPivotPosition(0.145)
                .andThen(new WaitCommand(0.25))
                .andThen(new SetPivotPosition(Constants.KirbyConstants.kExtendSetpoint))
                .andThen(new WaitCommand(0.25)))
                .repeatedly();
    }

}
