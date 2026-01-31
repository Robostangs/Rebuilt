package frc.robot.commands.PunisherCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class SetHoodDutyCycle extends Command {

  /*
   * This reads the input from the manip controller and sets the hood to that
   * The hood is first said to be in state of manual adjustment 
   * Then gets the input from the joystick constantly
   * When the code ends, the hood moves at {@code hoodDutyCycle} speed and its status is stopped
   */

  Turret hood;
  DoubleSupplier speed;

  public SetHoodDutyCycle(DoubleSupplier speed) {
    hood = Turret.getInstance();
    addRequirements(hood);
    this.speed = speed;
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
