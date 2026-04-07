package frc.robot.commands.DumperCommands.Flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Dumper.Flywheel;

public class ChargeUp extends Command {
    Flywheel shooter;
    DoubleSupplier targetRPS;
    boolean smort;

    public ChargeUp() {
        shooter = Flywheel.getInstance();
        addRequirements(shooter);
        // targetRPS = shooter.getFlywheelTargetVelocity();
        smort = true;
    }

    public ChargeUp(DoubleSupplier targetRPS) {
        shooter = Flywheel.getInstance();
        this.targetRPS = targetRPS;
        smort = false;
        addRequirements(shooter);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double currentTarget;

        if (smort) {
            currentTarget = shooter.getFlywheelTargetVelocity();
        }
        else{
            currentTarget = targetRPS.getAsDouble();
        }

        shooter.setVelocity(currentTarget);

        SmartDashboard.putNumber("Flywheel/Meters", Robot.getMetersFromTarget());

        shooter.postStatus("Charging to " + targetRPS + " rps");

    }

    @Override
    public void end(boolean interrupted) {
        shooter.postStatus("Charged");
        shooter.setFlywheelDC(0);
    }

}