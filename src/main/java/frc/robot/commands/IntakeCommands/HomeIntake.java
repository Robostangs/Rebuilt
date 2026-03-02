package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakePivot;

public class HomeIntake extends Command{
    IntakePivot intakePivot;
    Timer timer = new Timer();

        public HomeIntake(){

            intakePivot = IntakePivot.getInstance();
            addRequirements(intakePivot);
        }


    

    @Override
    public void initialize() {
        intakePivot.postStatus("Intake homing");
        intakePivot.setDC(Constants.KirbyConstants.kHomeDutyCycle);

    }

    @Override
    public void execute() {
        if(intakePivot.getStatorCurrent() > 50){
            timer.start();
        }
        else{
            timer.stop();
            timer.reset();
        }
        SmartDashboard.putNumber("Homing Time", timer.get());
    }
    

    @Override
    public void end(boolean interrupted) {
        intakePivot.setDC(0);
        intakePivot.setPosition(Constants.KirbyConstants.kHomePosition); 
        intakePivot.postStatus("Intake homed");

    }


    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Constants.HoodConstants.kHoodHomeTimeout);
    }
}
