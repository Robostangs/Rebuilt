package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kirby;

public class Swallow extends Command{
    Kirby kirby;

    public Swallow(){
        kirby = Kirby.getInstance();
    }
    
    public void initialize(){}

    
}
