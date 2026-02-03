package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kirby;

public class RetractIntake extends Command {
    
    Kirby kirby;

    public RetractIntake() {
        kirby = Kirby.getInstance();
        addRequirements(kirby);
    }
    @Override
    public void initialize() {
        kirby.setRetractPosition();
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
