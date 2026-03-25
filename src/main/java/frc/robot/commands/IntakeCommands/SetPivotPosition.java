package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakePivot;

/**
 * Spins the Kirby intake wheels at a given duty cycle while scheduled.
 */
public class SetPivotPosition extends Command {
    IntakePivot pivot;
    DoubleSupplier dutyCycle;
    double pos;
    /**
     * Create a new Swallow command that runs the Kirby wheels at the given duty
     * cycle.
     */
    public SetPivotPosition(Double position) {
        pivot = IntakePivot.getInstance(); 
        addRequirements(pivot);
        this.pos=position;
        
    }

    @Override
    public void initialize() {
        pivot.setExtenderPosition(pos);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return pivot.atTargetPosition();
    }
}
