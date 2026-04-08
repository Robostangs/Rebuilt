package frc.robot.FeederCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Dumper.Flywheel;

public class Feed extends Command{
     Flywheel shooter;
    DoubleSupplier targetRPS;

    public Feed(DoubleSupplier targetRPS) {
        shooter = Flywheel.getInstance();
        this.targetRPS = targetRPS;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double currentTarget;

 
            currentTarget = targetRPS.getAsDouble();
        

        shooter.setVelocity(currentTarget);


        shooter.postStatus("Charging to " + targetRPS + " rps");

    }

    @Override
    public void end(boolean interrupted) {
        shooter.postStatus("Charged");
    }

}
