package frc.robot.commands.TurretCommands.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Flywheel;

public class ChargeUp extends Command {
    Flywheel turretShooter;

    public ChargeUp() {
        turretShooter = Flywheel.getInstance();
        addRequirements(turretShooter);
    }

    @Override
    public void initialize() {
        // turretShooter.setShooterRPS(targetrps);
    }

    @Override
    public void execute() {
        double metersFromHub = 
        double targetrps = turretShooter.getTargetVelocity();
        turretShooter.setTargetVelocity();
        turretShooter.postStatus("Charging to " + targetrps + " rps");
    }

    @Override
    public void end(boolean interrupted) {
        turretShooter.postStatus("Charged");
    }

}
