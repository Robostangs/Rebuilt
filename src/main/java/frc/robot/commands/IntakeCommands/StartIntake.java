package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Kirby;

public class StartIntake extends Command {
    
    Kirby kirby;

    public StartIntake() {
        kirby = Kirby.getInstance();
        addRequirements(kirby);
    }

    @Override
    public void initialize() {
        kirby.setKirbyWheels(-1);
    }


    @Override
    public void end(boolean interrupted) {
        kirby.setKirbyWheels(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
