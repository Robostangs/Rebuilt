package frc.robot.commands.TurretCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.Hood;

public class setHoodPosition extends Command{
    Hood hood;
    Pose2d currentPose2d;

    public setHoodPosition(Pose2d currentPose2d){
        hood = Hood.getInstance();
        addRequirements(hood);
        this.currentPose2d = currentPose2d;
        }

        @Override
    public void initialize(){
        double hoodpos = hood.getHoodAngle(currentPose2d);
        hood.setHoodPosition(hoodpos);
    }
    
}
