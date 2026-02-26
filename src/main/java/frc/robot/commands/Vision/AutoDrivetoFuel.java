package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.MathUtil; // Added for clamping speeds

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.Stable.LimelightHelpers;
import frc.robot.Stable.LimelightHelpers.LimelightResults;
import frc.robot.Vision.VisionStangs;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Kirby;
import frc.robot.subsystems.Turret.Flywheel;

public class AutoDriveToFuel extends Command {
    private final CommandSwerveDrivetrain m_drive;
    private final Kirby m_intake;
    private final Flywheel m_flywheel;
    // RobotCentric is key: Forward is always where the camera points
    private final SwerveRequest.RobotCentric m_request = new SwerveRequest.RobotCentric();

    // --- SWERVE TUNING ---
    private final double kP_Steer = 0.04;    
    private final double kP_Drive = 0.05;    
    private final double kTargetTY = 18.0; // change values before comp   
    private final double kMaxSpeed = 2.5;    // Meters per second
    private final double kMaxRotation = 3.0; // Radians per second

    
    public AutoDriveToFuel() {
        // Use the singleton pattern to get the drivetrain instance
        m_drive = CommandSwerveDrivetrain.getInstance();
        m_intake = Kirby.getInstance();
        m_flywheel = Flywheel.getInstance();
        addRequirements(m_drive);
        addRequirements(m_intake);
        
    }

    @Override
    public void execute() {
        // I think it is a different name for the limelight
        LimelightResults results = LimelightHelpers.getLatestResults("Jarvis");
        LimelightHelpers.LimelightTarget_Detector fuel = results.targets_Detector[0];
        
        if (fuel.ta > 0.1) {
            double tx = fuel.tx;
            double ty = fuel.ty;
            //PID
            //Calculate Rotation
            double steerspeed = Constants.VisionConstants.m_steerController.calculate(fuel.tx, 0);
            //Calculate Forward
            double drivespeed = Constants.VisionConstants.m_driveController.calculate(fuel.ty, kTargetTY);
            //clamp the speed, this helps prevent it from flying away
            steerspeed = MathUtil.clamp(steerspeed, -kMaxRotation, kMaxRotation);
            drivespeed = MathUtil.clamp(drivespeed, -kMaxSpeed, kMaxSpeed);
            double h = 1;
            //forumla
            double distance = Math.sqrt(Math.pow(h/Math.cos(ty),2) - Math.pow(h,2));
            // if the limelight postion changes to not front facing, we have to put the rotational rate
            if ( distance > 0) {
                m_drive.setControl(
                m_request.withVelocityX(-tx) 
                         .withVelocityY(0));
                m_intake.setKirbyWheels(1.0);
                m_flywheel.setShooterRPS(3000.0);
            }

            //TODO: Implement proper PID 
            // Apply the request to your Swerve Drivetrain
            // TODO: Implement Intake (Feeder)
            // TODO: Implement Turret
        } else {
            m_drive.setControl(new SwerveRequest.Idle());
        }

       // Update vision data
    }

    @Override
    public boolean isFinished() {
        // Stop command once the ball is close enough to the target ty
        // TODO: Check network table if my intake is finished, if so, then end the command.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setControl(new SwerveRequest.Idle());
    }
}

