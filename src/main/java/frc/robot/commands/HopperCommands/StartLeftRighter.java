package frc.robot.commands.HopperCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

public class StartLeftRighter extends Command {
    
    Hopper hopper;


    public StartLeftRighter() {
        hopper = Hopper.getInstance();
        addRequirements(hopper);
    }
    
    @Override
    public void initialize() {
        hopper.setLeftRighterDutyCycle(Constants.HopperConstants.kLeftRighterDC);
    }

    @Override
    public void execute(){
        hopper.postLeftRighterStatus("Going at DC "+Constants.HopperConstants.kLeftRighterDC);
    }

    @Override
    public void end(boolean interrupted) {
        hopper.setLeftRighterDutyCycle(0);
        hopper.postLeftRighterStatus("Stopped");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
