package frc.robot.commands.ClimberCommands;

import java.lang.constant.Constable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class HomeClimber extends Command{
    Climber climber;
    double position;

    public HomeClimber(double position){
        climber = Climber.getInstance();
        this.position = position;
        addRequirements(climber);
    }

    @Override
    public void initialize(){
        climber.setClimberDutyCycle(Constants.ClimberConstants.kHomeDutyCycle);
        climber.postStatus("Climber homing");
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){
        climber.postStatus("Climber done homing.");
        climber.setClimberDutyCycle(0);
        climber.setClimberMechanismPosition(0);
    }

    @Override
    public boolean isFinished(){
            climber.postStatus("At zero position: "+climber.atZeroPosition());
            return climber.atZeroPosition();
    }



}
