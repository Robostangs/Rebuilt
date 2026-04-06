package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Intake.Kirby;

public class RetractIntake extends Command {

    IntakePivot pivot;
    Kirby kirby;

    public RetractIntake() {
        pivot = IntakePivot.getInstance();
        addRequirements( pivot);

    }

    @Override
    public void initialize() {
        pivot.setExtenderPosition(Constants.KirbyConstants.kRetractSetpoint);
    }

    public void execute() {
        pivot.setTargetPosition();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
