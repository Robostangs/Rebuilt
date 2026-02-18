
package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Stable.LimelightHelpers;
import frc.robot.Stable.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionStangs {
    private  PhotonCamera camera;
    private  Transform3d robotToCam;
    PhotonPoseEstimator ppe;
    private double ambiguity;
    private Pose2d currentPose;
    private double highestAmbiguity;
    private boolean isLL;
    private double timestampSeconds;

    private String CameraName;

    public VisionStangs(String LLname, Transform3d robotToCam) {
        camera = new PhotonCamera(LLname);
        this.robotToCam = robotToCam;
        this.CameraName = LLname;
        ppe = new PhotonPoseEstimator(Constants.kAprilTagFieldLayout, robotToCam);
        highestAmbiguity = -1;
        isLL = false;
    }


    public VisionStangs(String LLname) {
        this.CameraName = LLname;
        highestAmbiguity = -1;
        isLL = true;
    }

    // ...existing code...
    private void Photonperiodic() {

        double bigAmbiuity = 0d;

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

            visionEst = ppe.estimateCoprocMultiTagPose(result);

            if (visionEst.isEmpty()) {
                visionEst = ppe.estimateLowestAmbiguityPose(result);
            }

            for (var target : result.targets) {

                if (target.poseAmbiguity > bigAmbiuity) {
                    bigAmbiuity = target.poseAmbiguity;
                }
            }

            SmartDashboard.putNumber(camera.getName() + " biggest ambiguity", bigAmbiuity);
            ambiguity = bigAmbiuity;
        }

        if (bigAmbiuity < 0.2) {
            // Only use the estimate if present and valid
            visionEst.ifPresent(est -> {
                var pose2d = est.estimatedPose != null ? est.estimatedPose.toPose2d() : null;

                if (pose2d != null) {
                    timestampSeconds = est.timestampSeconds;
                    Robot.teleopField.getObject(camera.getName() + " Pose").setPose(pose2d);
                }
            });

        }

    }

    private void LLperiodic() {

        /*
         * Only seed ll4 with external yaw angle if not rotating fast, otherwise use
         * internal imu with mode 2
         */

        if (DriverStation.isDisabled()) {
            LimelightHelpers.SetIMUMode(CameraName, 1);
        } else {

            LimelightHelpers.SetRobotOrientation(CameraName,
                    CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(),
                    0d,
                    0d,
                    0d,
                    0d,
                    0d);

            LimelightHelpers.SetIMUMode(CameraName, 4);

        }

        LimelightHelpers.SetIMUAssistAlpha(CameraName, 0.4);

        if (DriverStation.isDisabled()) {
            NetworkTableInstance.getDefault().getTable(CameraName)
                    .getEntry("throttle-set").setNumber(200);
        } else {
            NetworkTableInstance.getDefault().getTable(CameraName)
                    .getEntry("throttle-set").setNumber(0);
        }
        if (!Robot.isSimulation()) {

            LimelightHelpers.PoseEstimate fourPoseEsti;
            if (DriverStation.isDisabled()) {

                fourPoseEsti = LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraName);

            } else if (CommandSwerveDrivetrain.getInstance().getPigeon2().getAngularVelocityZWorld()
                    .getValueAsDouble() < Constants.VisionConstants.kVisionAngularThreshold) {

                fourPoseEsti = LimelightHelpers
                        .getBotPoseEstimate_wpiBlue_MegaTag2(CameraName);

            } else {
                fourPoseEsti = LimelightHelpers
                        .getBotPoseEstimate_wpiBlue_MegaTag2(CameraName);

            }
            if (isPosegoo(fourPoseEsti)) {
                this.currentPose = fourPoseEsti.pose;   
                this.timestampSeconds = fourPoseEsti.timestampSeconds;
                Robot.teleopField.getObject("Limelight Four Pose").setPose(fourPoseEsti.pose);
                for(LimelightHelpers.RawFiducial fiducial:fourPoseEsti.rawFiducials){
                    if(fiducial.ambiguity > highestAmbiguity){
                        highestAmbiguity = fiducial.ambiguity;
                    }
                }
                
            }
        }
    }

    public double getTimeStamp(){
        return timestampSeconds;
    }

    public double getAmbiguity(){
        return highestAmbiguity;
    }
    public Pose2d getPose(){
        return currentPose;
    }
    public String getName(){
        return CameraName;
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

    public void periodic() {
        if (isLL) {
            LLperiodic();
        } else {
            Photonperiodic();
        }
    }
}