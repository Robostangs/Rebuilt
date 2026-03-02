package frc.robot.commands.IntakeCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Intake.Kirby;

public class ExtendIntake extends Command {
    
    Kirby kirby;
    IntakePivot pivot;

    public ExtendIntake() {
        kirby = Kirby.getInstance();
        pivot = IntakePivot.getInstance();
        addRequirements(kirby);
    }
    @Override
    public void initialize() {
        pivot.setExtenderPosition(Constants.KirbyConstants.kExtendSetpoint);
        kirby.setKirbyWheels(Constants.KirbyConstants.kirbyIntakeSpeed);
    }

    public void execute() {
        pivot.setTargetPosition();
    }


    @Override
    public void end(boolean interrupted) {

    }



}
