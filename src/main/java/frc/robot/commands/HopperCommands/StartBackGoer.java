package frc.robot.commands.HopperCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

public class StartBackGoer extends Command {
    
    Hopper hopper;

    public StartBackGoer() {
        hopper = Hopper.getInstance();
        addRequirements(hopper);
    }
    
    @Override
    public void initialize() {
        hopper.setBackGoerDutyCycle(Constants.HopperConstants.kBackGoerDC);
    }
    @Override
    public void execute(){
        hopper.postBackGoerStatus("Going at DC "+Constants.HopperConstants.kBackGoerDC);
    }

    @Override
    public void end(boolean interrupted) {
        hopper.setBackGoerDutyCycle(0);
        hopper.postBackGoerStatus("Stopped");
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
