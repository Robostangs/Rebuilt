package frc.robot.commands.DumperCommands.Hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Dumper.Hood;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.Hood;

public class setHoodPosition extends Command {
    Hood hood;

    public setHoodPosition() {
        hood = Hood.getInstance();
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.postStatus("Setting hood position");
    }

    @Override
    public void execute() {
        //same thing merge code with kabir
        double meterfromHub = Robot.getMetersFromTarget();
        double hoodpos = hood.getHoodAngle();

        hood.setMotionMagicTarget(hoodpos);
        hood.setMotionMagic();

        hood.postStatus("Setting hood position to: " + hoodpos);
        
        SmartDashboard.putNumber("Hood MFH", meterfromHub);

    }

    @Override
    public void end(boolean interrupted) {
        hood.postStatus("Hood position set");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
