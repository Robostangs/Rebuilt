package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Kirby;

public class StartIntake extends Command {
    
    Kirby kirby;
    DoubleSupplier speed;

    public StartIntake( DoubleSupplier speed) {
        kirby = Kirby.getInstance();
        this.speed = speed;
        addRequirements(kirby);
    }

    @Override
    public void initialize() {
        kirby.setKirbyWheels(speed.getAsDouble());
    }


    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
