package frc.robot.commands.DumperCommands.Hood;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Dumper.Hood;

public class HomeHood extends Command {
    /*
     * The elevator is sent to its home position once 
     * set at a speed of {@code kHoodHomeDutyCycle}
     * once it reaches the spot {@code kHomePosition}, it will stop moving
     */

    Hood hood;
    Timer timer = new Timer();


    public HomeHood() {
        hood = Hood.getInstance();

        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.postStatus("Hood homing");
        hood.setHoodDutyCycle(Constants.HoodConstants.kHoodHomeDutyCycle);
        // hood.setHoodStatorCurrentConfig();

    }

    @Override
    public void execute() {
        if(hood.getStatorCurrent() < -13){
            timer.start();
        }
        else{
            timer.stop();
            timer.reset();
        }
        SmartDashboard.putNumber("Homing Time", timer.get());
    }
    

    @Override
    public void end(boolean interrupted) {
        hood.setHoodDutyCycle(0);
        hood.setHoodPosition(Constants.DumperConstants.kHomePosition);
        hood.postStatus("Hood homed");
        hood.setIsHome(true);
        // hood.configureHoodMotor();

    }


    @Override
    public boolean isFinished() {
        return 
        // hood.getLimitSwitch() || 
        timer.hasElapsed(Constants.HoodConstants.kHoodHomeTimeout);
    }
}
