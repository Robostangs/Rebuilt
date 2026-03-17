package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command {
    CommandSwerveDrivetrain drivetrain;

    SwerveRequest.FieldCentricFacingAngle driveRequest;

    Supplier<Double> translateX, translateY;
    Supplier<Boolean> babyOnBoard;
    Supplier<Rotation2d> getTargetRotation;

    Pose2d targetPose;

    /**
     * Aligns the robot to the {@code kPassPoseRed} or {@code kPassPoseBlue}
     * 
     * @param translateX           the forward to backward movement of the robot
     * @param translateY           the right to left movement of the robot
     * @param howManyBabiesOnBoard 1 - the value of how much to slow down (right
     *                             trigger axis)
     */
    public AutoAlign(Supplier<Double> translateX, Supplier<Double> translateY, Supplier<Boolean> babyOnBoard) {

        drivetrain = CommandSwerveDrivetrain.getInstance();

        this.addRequirements(drivetrain);

        this.translateX = translateX;
        this.translateY = translateY;
        this.babyOnBoard = babyOnBoard;
        this.targetPose = Constants.FieldConstants.HubPosition;
        // !Robot.isRed() ? Constants.FieldConstants.HubPosition :
        // FlippingUtil.flipFieldPose(Constants.FieldConstants.HubPosition);

        getTargetRotation = () -> {

            return Rotation2d
                    .fromRadians(Math.atan2(
                            drivetrain.getState().Pose.getY() - targetPose.getY(),
                            drivetrain.getState().Pose.getX() - targetPose.getX()));
        };
    }

    @Override
    public void initialize() {

        PhoenixPIDController tunePID = new PhoenixPIDController(6,1,1);
        tunePID.enableContinuousInput(-Math.PI, Math.PI);

        driveRequest = new SwerveRequest.FieldCentricFacingAngle();
        driveRequest.HeadingController = tunePID;
    

        // this is for tuning and now we can tune the PID controller
        SmartDashboard.putData("Auto Align PID", driveRequest.HeadingController);
        drivetrain.postStatus("Auto Aligning");

        driveRequest.Deadband =
        Constants.TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.05;
        driveRequest.RotationalDeadband =
        Constants.TunerConstants.kAngularRateAt12Volts * 0.05;
    }

    @Override
    public void execute() {
        // getTargetRotation = () -> {

        // return Rotation2d
        // .fromRadians((Math.atan2(
        // drivetrain.getState().Pose.getY() - targetPose.getY(),
        // drivetrain.getState().Pose.getX() - targetPose.getX())));
        // };

        SmartDashboard.putNumber("Drive theta",
                Math.atan2(
                        drivetrain.getState().Pose.getY(),
                        drivetrain.getState().Pose.getX()));

        SmartDashboard.putNumber("Drive current theta",
                Math.atan2(
                        drivetrain.getState().Pose.getY() - targetPose.getY(),
                        drivetrain.getState().Pose.getX() - targetPose.getX()));

        if (Math.abs(getTargetRotation.get().getDegrees()) >= Rotation2d.kPi.getDegrees()) {
            driveRequest.TargetDirection = Rotation2d.kPi;
        } else {
            driveRequest.TargetDirection = getTargetRotation.get();

        }

        driveRequest
                .withVelocityX(-translateX.get()
                        * (Constants.TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()))
                .withVelocityY(-translateY.get()
                        * Constants.TunerConstants.kSpeedAt12Volts.baseUnitMagnitude());

        drivetrain.setControl(driveRequest);
        drivetrain.postStatus("Currently aligning");

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.postStatus("Aligned");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
