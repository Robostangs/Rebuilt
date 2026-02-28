package frc.robot.commands.TurretCommands.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.Flywheel;
import frc.robot.subsystems.Turret.Turret;

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
        double metersFromHub = Turret.getInstance().getTurretPose().getTranslation().getDistance(Constants.FieldConstants.HubPosition.getTranslation());
        double targetrps = turretShooter.getTargetVelocity(metersFromHub);
        turretShooter.postStatus("Charging to " + targetrps + " rps");
    }

    @Override
    public void end(boolean interrupted) {
        turretShooter.postStatus("Charged");
    }

}
