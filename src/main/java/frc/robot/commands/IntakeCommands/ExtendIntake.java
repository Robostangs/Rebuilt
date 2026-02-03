package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Kirby;

public class ExtendIntake extends Command {
    
    Kirby kirby;

    public ExtendIntake() {
        kirby = Kirby.getInstance();
        addRequirements(kirby);
    }
    @Override
    public void initialize() {
        kirby.setExtendPosition();
    }


    @Override
    public void end(boolean interrupted) {
        kirby.setIntakeExtenderBrake();

    }

    @Override
    public boolean isFinished() {
        return kirby.intakeExtendAtTargetPosition();
    }

}
