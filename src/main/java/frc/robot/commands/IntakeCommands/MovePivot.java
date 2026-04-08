package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakePivot;

/**
 * Spins the Kirby intake wheels at a given duty cycle while scheduled.
 */
public class MovePivot extends Command {
    private final IntakePivot pivot;
    private final DoubleSupplier dutyCycle;

    /**
     * Create a new Swallow command that runs the Kirby wheels at the given duty
     * cycle.
     */
    public MovePivot(DoubleSupplier dutyCycle) {
        pivot = IntakePivot.getInstance();
        this.dutyCycle = dutyCycle;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        pivot.setDC(dutyCycle.getAsDouble());

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // This command runs until explicitly cancelled.
        return false;
    }
}
