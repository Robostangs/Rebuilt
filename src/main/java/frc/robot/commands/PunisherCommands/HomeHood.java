package frc.robot.commands.PunisherCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.Turret;

public class HomeHood extends Command {
    /*
     * The elevator is sent to its home position once 
     * set at a speed of {@code kHoodHomeDutyCycle}
     * once it reaches the spot {@code kHomePosition}, it will stop moving
     */

    Turret hood;

    public HomeHood() {
        hood = Turret.getInstance();

        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.postStatus("elevator homing");
        hood.setHoodDutyCycle(Constants.TurretConstants.kHoodHomeDutyCycle);

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        hood.setHoodDutyCycle(0);
        hood.setHoodPosition(Constants.TurretConstants.kHomePosition);
        hood.postStatus("elevator homed");

    }

    public boolean isFinished() {
        return hood.getLimitSwitch();
    }
}
