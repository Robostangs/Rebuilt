package frc.robot.commands.TurretCommands.Hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Hood;
import frc.robot.subsystems.Turret.Turret;

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
        double meterfromHub =  Turret.getInstance().getMetersFromHub();
        double hoodpos = hood.getHoodAngle(meterfromHub);

        hood.setMotionMagicTarget(hoodpos);


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
