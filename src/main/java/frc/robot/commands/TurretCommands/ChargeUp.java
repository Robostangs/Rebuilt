package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.TurretShooter;

public class ChargeUp extends Command {
    double targetrps;
    TurretShooter turretShooter;

    public ChargeUp(double targetrps) {
        this.targetrps = targetrps;
        turretShooter = TurretShooter.getInstance();
        addRequirements(turretShooter);
    }

    @Override
    public void initialize() {
        turretShooter.setShooterRPS(targetrps);
    }

    @Override
    public void execute() {
        turretShooter.postStatus("Charging to " + targetrps + " rps");
    }

    @Override
    public void end(boolean interrupted) {
        turretShooter.postStatus("Charged");
    }

}
