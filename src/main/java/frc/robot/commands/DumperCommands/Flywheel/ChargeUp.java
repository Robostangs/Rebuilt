package frc.robot.commands.DumperCommands.Flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Dumper.Flywheel;

public class ChargeUp extends Command {
    Flywheel shooter;

    public ChargeUp() {
        shooter = Flywheel.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setCanShoot(true);
    }

    @Override
    public void execute() {
        double targetrps = shooter.getFlywheelTargetVelocity();
        shooter.setFlywheelVelocity(targetrps);

        SmartDashboard.putNumber("Hood MFH", Robot.getMetersFromTarget());

        shooter.postStatus("Charging to " + targetrps + " rps");
    }

    @Override
    public void end(boolean interrupted) {
        shooter.postStatus("Charged");
        shooter.setFlywheelVelocity(0);
    }

}