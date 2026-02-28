package frc.robot.commands.TurretCommands.Flywheel;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    }

    @Override
    public void execute() {
        Supplier<Double> meterfromHub = () -> Turret.getInstance().getMetersFromHub();
        double targetrps = turretShooter.getTargetVelocity(meterfromHub.get());
        turretShooter.setShooterRPS(targetrps);

        SmartDashboard.putNumber("Hood MFH", meterfromHub.get());

        turretShooter.postStatus("Charging to " + targetrps + " rps");
    }

    @Override
    public void end(boolean interrupted) {
        turretShooter.postStatus("Charged");
        turretShooter.setShooterRPS(0);
    }

}
