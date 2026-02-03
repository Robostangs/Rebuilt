package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Kirby;

public class OutTake extends Command {
    
    Kirby kirby;

    public OutTake() {
        kirby = Kirby.getInstance();
        addRequirements(kirby);
    }
    @Override
    public void initialize() {
        kirby.setKirbyWheels(-Constants.KirbyConstants.kirbyIntakeSpeed);
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
