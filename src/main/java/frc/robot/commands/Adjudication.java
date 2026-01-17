package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Punisher;

public class Adjudication extends Command {

    Punisher punisher;
    // DoubleSupplier speed;

    public Adjudication( ) {
        this.punisher = Punisher.getInstance();
        // this.speed = speed;
        addRequirements(punisher);
    }

    @Override
    public void initialize() {
            punisher.setPunisherdutycycle(-0.4);
            punisher.setJigsawdutycycle(-0.25);

    }

    @Override
    public void execute(){

    }
    @Override
    public void end(boolean interrupted) {
        punisher.setPunisherdutycycle(0);
        punisher.setJigsawdutycycle(0);
    }
}