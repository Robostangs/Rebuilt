package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Dumper.Flywheel;

public class Index extends Command {
    Feeder feeder;
    boolean isDumb;

    public Index() {
        feeder = Feeder.getInstance();
        addRequirements(feeder);
        this.isDumb = isDumb;
    }

    @Override
    public void initialize() {

        feeder.postStatus("Index started: " + Constants.FeederConstants.kFeedDutyCycle);
    }

    public void execute() {

     

    if (Flywheel.getInstance().shooterAtTargetVelocity() ) {
        feeder.setDutyCycle(Constants.FeederConstants.kFeedDutyCycle);
        }
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
