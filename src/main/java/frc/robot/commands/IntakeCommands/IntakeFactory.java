package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FeederCommands.setFeederSpeed;
import frc.robot.commands.HopperCommands.RunHopper;

public class IntakeFactory {

    public static Command TrueSpit() {
        return new setFeederSpeed(() -> -1).alongWith(new setKirbyWheels(() -> -1))
                .alongWith(new RunHopper(() -> -0.5));
    }

    public static Command ballStuck() {
        return new setFeederSpeed(() -> -1).alongWith(new setKirbyWheels(() -> -1))
                .alongWith(new RunHopper(() -> -0.5));
    }

    public static Command ExtendIntake() {
        return new setPivotPosition(Constants.KirbyConstants.kExtendSetpoint)
            .alongWith(new setKirbyWheels(() -> 1))
            .withName("Extend Intake Command");
    }

    public static Command RetractIntake() {
        return new setPivotPosition(Constants.KirbyConstants.kRetractSetpoint)
        .withName("Retract Intake Command");
    }

    public static Command OscillateIntake() {
        return (new setPivotPosition(0.145)
                .andThen(new WaitCommand(0.25))
                .andThen(new setPivotPosition(Constants.KirbyConstants.kExtendSetpoint))
                .andThen(new WaitCommand(0.25)))
                .repeatedly()
                .withName("Oscillate Intake Command");
    }

}
