package frc.robot.commands.HopperCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

public class spitHopper extends Command {
     Hopper hopper;


    public spitHopper() {
        hopper = Hopper.getInstance();
        addRequirements(hopper);
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute(){
        hopper.setBackGoerDutyCycle(Constants.HopperConstants.kBackGoerDC);

    }

    @Override
    public void end(boolean interrupted) {
        hopper.setLeftRightDutyCycle(0);
        hopper.setBackGoerDutyCycle(0);
        hopper.postLeftRighterStatus("Stopped");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
