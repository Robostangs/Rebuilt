package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Punisher;

public class Adjudication extends Command {

    Punisher punisher;

    public Adjudication( ) {
        this.punisher = Punisher.getInstance();
        addRequirements(punisher);
    }

    @Override
    public void initialize() {
            punisher.setPunisherdutycycle(-0.4);
            punisher.setMicrodutycycle(-0.25);

    }

    @Override
    public void execute(){
    }
    @Override
    public void end(boolean interrupted) {
        punisher.setPunisherdutycycle(0);
        punisher.setMicrodutycycle(0);
    }
}