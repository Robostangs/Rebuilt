package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.MathUtil; // Added for clamping speeds

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Vision.VisionStangs;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoDriveToFuel extends Command {
    private final CommandSwerveDrivetrain m_drive;
    
    // RobotCentric is key: Forward is always where the camera points
    private final SwerveRequest.RobotCentric m_request = new SwerveRequest.RobotCentric();

    // --- SWERVE TUNING ---
    private final double kP_Steer = 0.04;    
    private final double kP_Drive = 0.05;    
    private final double kTargetTY = 18.0;   
    private final double kMaxSpeed = 2.5;    // Meters per second
    private final double kMaxRotation = 3.0; // Radians per second

    public AutoDriveToFuel() {
        // Use the singleton pattern to get the drivetrain instance
        m_drive = CommandSwerveDrivetrain.getInstance();
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTA("limelight")) {
            double tx = LimelightHelpers.getTX("limelight");
            double ty = LimelightHelpers.getTY("limelight");

            //pid
            double steerError = -tx; 
            double driveError = kTargetTY - ty;

            double forwardVelocity = MathUtil.clamp(driveError * kP_Drive, -kMaxSpeed, kMaxSpeed);
            double rotationVelocity = MathUtil.clamp(steerError * kP_Steer, -kMaxRotation, kMaxRotation);

            // Apply the request to your Swerve Drivetrain
            m_drive.setControl(
                m_request.withVelocityX(forwardVelocity) 
                         .withVelocityY(0) 
                         .withRotationalRate(rotationVelocity)
            );
        } else {
            m_drive.setControl(new SwerveRequest.Idle());
        }

       // Update vision data
    }

    @Override
    public boolean isFinished() {
        // Stop command once the ball is close enough to the target ty
        return LimelightHelpers.getTA("limelight") && 
               (LimelightHelpers.getTY("limelight") >= kTargetTY - 1.0);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setControl(new SwerveRequest.Idle());
    }
}

class LimelightHelpers {
    public static boolean getTA(String tableName) {
        return NetworkTableInstance.getDefault().getTable(tableName).getEntry("ta").getDouble(0.0) == 1.0;
    }
    public static double getTX(String tableName) {
        return NetworkTableInstance.getDefault().getTable(tableName).getEntry("tx").getDouble(0.0);
    }
    public static double getTY(String tableName) {
        return NetworkTableInstance.getDefault().getTable(tableName).getEntry("ty").getDouble(0.0);
    }
}