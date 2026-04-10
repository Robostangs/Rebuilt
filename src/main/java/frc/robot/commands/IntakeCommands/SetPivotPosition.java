package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakePivot;


public class setPivotPosition extends Command {
    IntakePivot pivot;
    double pos;

  
    public setPivotPosition(double position) {
        pivot = IntakePivot.getInstance();
        this.pos = position;

        addRequirements(pivot);

    }

    @Override
    public void initialize() {
        pivot.setExtenderPosition(pos);
        pivot.postStatus("going to: " + pos);
    }

    @Override
    public void execute() {
        pivot.setTargetPosition();

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return pivot.atTargetPosition();
    }
}
