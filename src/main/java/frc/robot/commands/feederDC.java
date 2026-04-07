package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class feederDC extends Command {
    double dutyCycle;

    public feederDC(double dutyCycle) {
        this.dutyCycle = dutyCycle;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Index feederIndex = new Index();
        feederIndex.execute();
    }

    @Override
    public void end(boolean interrupted) {
        Undex feederUndex = new Undex();
        feederUndex.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
