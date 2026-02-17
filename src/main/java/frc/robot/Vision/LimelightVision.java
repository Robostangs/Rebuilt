package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Stable.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LimelightVision {
    private String LLname;
    private double ambiguity;
    private Pose2d currentPose;
    private double highestAmbiguity;

    public LimelightVision(String LLname) {
        this.LLname = LLname;
        highestAmbiguity = -1;
    }

    public void configLL4() {

        /*
         * Only seed ll4 with external yaw angle if not rotating fast, otherwise use
         * internal imu with mode 2
         */

        if (DriverStation.isDisabled()) {
            LimelightHelpers.SetIMUMode(LLname, 1);
        } else {

            LimelightHelpers.SetRobotOrientation(LLname,
                    CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(),
                    0d,
                    0d,
                    0d,
                    0d,
                    0d);

            LimelightHelpers.SetIMUMode(LLname, 4);

        }

        LimelightHelpers.SetIMUAssistAlpha(LLname, 0.4);

        if (DriverStation.isDisabled()) {
            NetworkTableInstance.getDefault().getTable(LLname)
                    .getEntry("throttle-set").setNumber(200);
        } else {
            NetworkTableInstance.getDefault().getTable(LLname)
                    .getEntry("throttle-set").setNumber(0);
        }
        if (!Robot.isSimulation()) {

            LimelightHelpers.PoseEstimate fourPoseEsti;
            if (DriverStation.isDisabled()) {

                fourPoseEsti = LimelightHelpers.getBotPoseEstimate_wpiBlue(LLname);

            } else if (CommandSwerveDrivetrain.getInstance().getPigeon2().getAngularVelocityZWorld()
                    .getValueAsDouble() < Constants.VisionConstants.kVisionAngularThreshold) {

                fourPoseEsti = LimelightHelpers
                        .getBotPoseEstimate_wpiBlue_MegaTag2(LLname);

            } else {
                fourPoseEsti = LimelightHelpers
                        .getBotPoseEstimate_wpiBlue_MegaTag2(LLname);

            }
            if (isPosegoo(fourPoseEsti)) {
                CommandSwerveDrivetrain.getInstance().addVisionMeasurement(fourPoseEsti.pose, fourPoseEsti.timestampSeconds);
                this.currentPose = fourPoseEsti.pose;   
                Robot.teleopField.getObject("Limelight Four Pose").setPose(fourPoseEsti.pose);

                for(LimelightHelpers.RawFiducial fiducial:fourPoseEsti.rawFiducials){
                    if(fiducial.ambiguity > highestAmbiguity){
                        highestAmbiguity = fiducial.ambiguity;
                    }
                }
                
            }
        }


    }

    public double getAmbiguity(){
        return highestAmbiguity;
    }
    public Pose2d getPose(){
        return currentPose;
    }

    private boolean isPosegoo(LimelightHelpers.PoseEstimate yoPoseEsti) {
        if (Robot.isSimulation()) {
            SmartDashboard.putString("Vision/LL4_RejectReason", "Simulation mode");
            return false;
        }
        if (yoPoseEsti == null) {
            SmartDashboard.putString("Vision/LL4_RejectReason", "Pose null");
            return false;
        }
        if (yoPoseEsti.tagCount <= 0) {
            SmartDashboard.putString("Vision/LL4_RejectReason", "Not enough tags: " + yoPoseEsti.tagCount);
            return false;
        }

        if (!RobotContainer.useVision) {
            SmartDashboard.putString("Vision/LL4_RejectReason", "useVision disabled");
        } else {
            SmartDashboard.putString("Vision/LL4_RejectReason", "Accepted");
        }

        return RobotContainer.useVision;
    }
}
