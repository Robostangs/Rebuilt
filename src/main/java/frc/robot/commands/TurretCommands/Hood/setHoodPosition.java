package frc.robot.commands.TurretCommands.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
            double metersFromHub = Turret.getInstance().getTurretPose().getTranslation().getDistance(Constants.FieldConstants.HubPosition.getTranslation());
            double hoodpos = hood.getHoodAngle(metersFromHub);
            hood.setHoodPosition(hoodpos);
            hood.postStatus("Setting hood position to: "+ hoodpos);

        }
    
        @Override
        public void end(boolean interrupted) {
            hood.postStatus("Hood position set");
        }
    
        @Override
        public boolean isFinished() {
            return true;
        }
}
