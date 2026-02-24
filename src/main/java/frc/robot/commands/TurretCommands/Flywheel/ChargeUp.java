package frc.robot.commands.TurretCommands.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Flywheel;

public class ChargeUp extends Command {
    double targetrps;
    Flywheel turretShooter;

    public ChargeUp(double targetrps) {
        this.targetrps = targetrps;
        turretShooter = Flywheel.getInstance();
        addRequirements(turretShooter);
    }

    @Override
    public void initialize() {
        // turretShooter.setShooterRPS(targetrps);
    }

    @Override
    public void execute() {
        turretShooter.setMotionMagic();
        turretShooter.postStatus("Charging to " + targetrps + " rps");
    }

    @Override
    public void end(boolean interrupted) {
        turretShooter.postStatus("Charged");
    }

}
