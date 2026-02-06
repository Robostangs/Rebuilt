package frc.robot.commands.TurretCommands;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.Hood;

public class HomeHood extends Command {
    /*
     * The elevator is sent to its home position once 
     * set at a speed of {@code kHoodHomeDutyCycle}
     * once it reaches the spot {@code kHomePosition}, it will stop moving
     */

    Hood hood;


    public HomeHood() {
        hood = Hood.getInstance();

        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.postStatus("Hood homing");
        hood.setHoodDutyCycle(Constants.HoodConstants.kHoodHomeDutyCycle);

    }

    @Override
    public void execute() {
    }
    

    @Override
    public void end(boolean interrupted) {
        hood.setHoodDutyCycle(0);
        hood.setHoodPosition(Constants.TurretConstants.kHomePosition);
        hood.postStatus("Hood homed");

    }


    @Override
    public boolean isFinished() {
        return hood.getLimitSwitch();
    }
}
