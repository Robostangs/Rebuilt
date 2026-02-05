package frc.robot.commands.IntakeCommands;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Kirby;

public class ExtendIntake extends Command {
    
    Kirby kirby;

    public ExtendIntake() {
        kirby = Kirby.getInstance();
        addRequirements(kirby);
    }
    @Override
    public void initialize() {
        kirby.setExtenderPosition(Constants.KirbyConstants.kExtendSetpoint);
        kirby.setKirbyWheels(Constants.KirbyConstants.kirbyIntakeSpeed);
    }


    @Override
    public void end(boolean interrupted) {

    }



}
