package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class SetLevel extends Command{
    Climber climber;
    double position;

    public SetLevel(double position){
        climber = Climber.getInstance();
        this.position = position;
        addRequirements(climber);
    }

    @Override
    public void initialize(){
        climber.setClimberPosition(position);
        climber.postStatus("Climber going to: " + position);
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        climber.postStatus("Climber done.");
    }

    @Override
    public boolean isFinished(){
            climber.postStatus("Can't deploy");
            return climber.isStowed();
    }



}
