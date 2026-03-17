package frc.robot.commands.DumperCommands.Hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Dumper.Hood;

public class setHoodPositionTrench extends Command {
    Hood hood;

    public setHoodPositionTrench() {
        hood = Hood.getInstance();

        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.postStatus("Setting hood position");
    }

    @Override
    public void execute() {
        double hoodpos = 0;
        hoodpos = .4;

        hood.setMotionMagicTarget(hoodpos);

        hood.setMotionMagic();

        hood.postStatus("Setting Trench hood position to: " + hoodpos);


    }

    @Override
    public void end(boolean interrupted) {
        hood.postStatus("Hood position set Trench");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
