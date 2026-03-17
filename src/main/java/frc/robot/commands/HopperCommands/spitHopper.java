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
        hopper.setRollerDutyCycle(-Constants.HopperConstants.kRollerDC);

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
