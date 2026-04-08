package frc.robot.commands.HopperCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Dumper.Flywheel;

public class RunHopper extends Command {

    Hopper hopper;
    DoubleSupplier speed;

    public RunHopper(DoubleSupplier speed) {
        hopper = Hopper.getInstance();
        this.speed = speed;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        hopper.postRollerStatus("Going at DC " + Constants.HopperConstants.kRollerDC);

        hopper.setRollerDutyCycle(speed.getAsDouble());

    }

    @Override
    public void end(boolean interrupted) {
        hopper.postRollerStatus("Stopped");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
