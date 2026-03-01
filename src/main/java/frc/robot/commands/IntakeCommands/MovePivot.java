package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kirby;

/**
 * Spins the Kirby intake wheels at a given duty cycle while scheduled.
 */
public class MovePivot extends Command {
    private final Kirby kirby;
    private final DoubleSupplier dutyCycle;

    /**
     * Create a new Swallow command that runs the Kirby wheels at the given duty
     * cycle.
     */
    public MovePivot(DoubleSupplier dutyCycle) {
        this.kirby = Kirby.getInstance();
        this.dutyCycle = dutyCycle;
        addRequirements(kirby);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        kirby.setDC(dutyCycle.getAsDouble());

    }

    @Override
    public void end(boolean interrupted) {
        kirby.setKirbyWheels(0.0);
    }

    @Override
    public boolean isFinished() {
        // This command runs until explicitly cancelled.
        return false;
    }
}
