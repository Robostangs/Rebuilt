package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Punisher;

public class Adjudication extends Command {

    Punisher punisher;

    public Adjudication() {
        this.punisher = Punisher.getInstance();
        addRequirements(punisher);
    }

    @Override
    public void initialize() {
        punisher.setPunisherdutycycle(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        punisher.setPunisherdutycycle(0);
    }
}