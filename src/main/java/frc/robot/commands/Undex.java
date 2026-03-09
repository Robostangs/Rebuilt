
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Turret.Flywheel;

public class Undex extends Command {
    Feeder feeder;

    public Undex() {
        feeder = Feeder.getInstance();
        addRequirements(feeder);
    }

    @Override
    public void initialize() {

        feeder.postStatus("Undex started: " + -1);
    }

    public void execute() {
        if (Flywheel.getInstance().shooterAtTargetVelocity()) {
            feeder.setDutyCycle(-1);

        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setDutyCycle(0.0);
        feeder.postStatus("Undex ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
