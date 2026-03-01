package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;

public class Index extends Command {
    Feeder feeder;

    public Index() {
        feeder = Feeder.getInstance();
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.setDutyCycle(Constants.FeederConstants.kFeedDutyCycle);

        feeder.postStatus("Index started: " + Constants.FeederConstants.kFeedDutyCycle);
    }

 
    @Override
    public void end(boolean interrupted) {
        feeder.setDutyCycle(0.0);
        feeder.postStatus("Index ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
