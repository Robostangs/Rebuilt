package frc.robot.commands.DumperCommands.Flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Dumper.Flywheel;

public class setFlywheelDutyCycle extends Command {
    Flywheel flywheel;
    DoubleSupplier speed;

    public setFlywheelDutyCycle(DoubleSupplier speed){
        this.speed = speed;
        flywheel = Flywheel.getInstance();
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        flywheel.setFlywheelDC(speed.getAsDouble());
        flywheel.setControl();
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setFlywheelDC(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
