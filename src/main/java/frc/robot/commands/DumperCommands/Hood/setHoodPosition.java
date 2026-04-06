package frc.robot.commands.DumperCommands.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Dumper.Hood;

public class setHoodPosition extends Command {
    Hood hood;
    double hoodpos;
    boolean smort;

    public setHoodPosition() {
        hood = Hood.getInstance();
        this.smort = true;
        addRequirements(hood);
    }

    public setHoodPosition(double hoodpos) {
        this.hoodpos = hoodpos;
        hood = Hood.getInstance();
        this.smort = false;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.postStatus("Setting hood position");
    }

    @Override
    public void execute() {


        if(smort) {
            hoodpos = hood.getHoodAngle();
        }

        hood.setMotionMagicTarget(hoodpos);
        hood.setMotionMagic();

        hood.postStatus("Setting hood position to: " + hoodpos);


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
