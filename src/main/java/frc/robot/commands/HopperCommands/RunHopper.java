package frc.robot.commands.HopperCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Dumper.Flywheel;

public class RunHopper extends Command {

    Hopper hopper;

    public RunHopper() {
        hopper = Hopper.getInstance();
        addRequirements(hopper);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        hopper.postRollerStatus("Going at DC " + Constants.HopperConstants.kRollerDC);
        if (Flywheel.getInstance().shooterAtTargetVelocity()) {

            hopper.setRollerDutyCycle(Constants.HopperConstants.kRollerDC);
        }

    }

    @Override
    public void end(boolean interrupted) {
        hopper.setRollerDutyCycle(0);
        hopper.postRollerStatus("Stopped");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
