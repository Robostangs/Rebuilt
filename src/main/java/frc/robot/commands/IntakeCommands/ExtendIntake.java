package frc.robot.commands.IntakeCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Intake.Kirby;

public class ExtendIntake extends Command {
    
    Kirby kirby;
    IntakePivot pivot;
    boolean isBump;

    public ExtendIntake(boolean isBump) {
        kirby = Kirby.getInstance();
        pivot = IntakePivot.getInstance();
        this.isBump = isBump;
        addRequirements(kirby);
    }
    @Override
    public void initialize() {
        if(isBump){
            pivot.setExtenderPosition(0.05);
        }
        else{
        pivot.setExtenderPosition(Constants.KirbyConstants.kExtendSetpoint);
    }
        kirby.setKirbyWheels(Constants.KirbyConstants.kirbyIntakeSpeed);
        kirby.postStatus("Wheels going at: "+Constants.KirbyConstants.kirbyIntakeSpeed );
                pivot.postStatus("Extended");

    }

    public void execute() {
        pivot.setTargetPosition();
    }


    @Override
    public void end(boolean interrupted) {
        kirby.setKirbyWheels(0);
                pivot.postStatus("Extended End");

        
    }
    @Override
    public boolean isFinished() {
        return false;
    }


}
