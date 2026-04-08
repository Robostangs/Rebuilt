package frc.robot.commands.DumperCommands.Flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Dumper.Flywheel;
import frc.robot.subsystems.Dumper.Hood;

public class setFlywheelDutyCycle extends Command{
   
  Flywheel flywheel;
  DoubleSupplier speed;

  public setFlywheelDutyCycle(DoubleSupplier speed) {
    flywheel = Flywheel.getInstance();
    this.speed = speed;
    addRequirements(flywheel);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    flywheel.setFlywheelDC(speed.getAsDouble());
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
