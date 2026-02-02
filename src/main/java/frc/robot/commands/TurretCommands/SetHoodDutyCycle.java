package frc.robot.commands.TurretCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Hood;

public class SetHoodDutyCycle extends Command {

  /*
   * This reads the input from the manip controller and sets the hood to that
   * The hood is first said to be in state of manual adjustment
   * Then gets the input from the joystick constantly
   * When the code ends, the hood moves at {@code hoodDutyCycle} speed and its
   * status is stopped
   */

  Hood hood;
  DoubleSupplier speed;

  public SetHoodDutyCycle(DoubleSupplier speed) {
    hood = Hood.getInstance();
    this.speed = speed;
    addRequirements(hood);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    hood.setHoodDutyCycle(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    hood.setHoodDutyCycle(0);

  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
