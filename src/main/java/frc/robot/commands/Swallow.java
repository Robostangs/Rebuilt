package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kirby;

/**
 * Spins the Kirby intake wheels at a given duty cycle while scheduled.
 */
public class Swallow extends Command {
    private final Kirby kirby;
    private final double dutyCycle;

    /**
     * Create a new Swallow command that runs the Kirby wheels at the given duty
     * cycle.
     */
    public Swallow(double dutyCycle) {
        this.kirby = Kirby.getInstance();
        this.dutyCycle = dutyCycle;
        // Declare subsystem dependency so scheduler won't run conflicting commands.
        addRequirements(kirby);
    }

    @Override
    public void initialize() {
        kirby.setKirbyWheels(dutyCycle);

    }

    @Override
    public void execute() {
        // Set motor output to the provided duty cycle.
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the wheels when the command ends or is interrupted.
        kirby.setKirbyWheels(0.0);
    }

    @Override
    public boolean isFinished() {
        // This command runs until explicitly cancelled.
        return false;
    }
}
