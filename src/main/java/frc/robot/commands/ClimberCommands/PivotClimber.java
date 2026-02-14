package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class PivotClimber extends Command {
    Climber climber;
    double position;

    public PivotClimber(double position){
        climber = Climber.getInstance();
        this.position = position;
        addRequirements(climber);
    }

    @Override
    public void initialize(){
        climber.setClimberPiviotPosition(position);
        climber.postStatus("Climber going to: " + position);
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        climber.postStatus("climber done");
    }

    @Override
    public boolean isFinished(){
        return climber.isPivotAtTarget();
    }

}
  
