
package frc.robot.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision {
    private final PhotonCamera camera;
    private final Transform3d robotToCam;
    PhotonPoseEstimator ppe;

    public PhotonVision(String photonCamName, Transform3d robotToCam) {
        camera = new PhotonCamera(photonCamName);
        this.robotToCam = robotToCam;
        ppe = new PhotonPoseEstimator(Constants.kAprilTagFieldLayout, robotToCam);
    }

   // ...existing code...
    public void PhotonPeriodic() {

         double biggestAmbiguity = -1d;

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

            visionEst = ppe.estimateCoprocMultiTagPose(result);

            if (visionEst.isEmpty()) {
                visionEst = ppe.estimateLowestAmbiguityPose(result);
            }

            for (var target : result.targets) {
                
                if(target.poseAmbiguity > biggestAmbiguity){
                    biggestAmbiguity = target.poseAmbiguity;
                }
            }

            SmartDashboard.putNumber(camera.getName()+" biggest ambiguity", biggestAmbiguity);
        }

        if (biggestAmbiguity > 0 && biggestAmbiguity  < 0.2) {
            // Only use the estimate if present and valid
            visionEst.ifPresent(est -> {
                var pose2d = est.estimatedPose != null ? est.estimatedPose.toPose2d() : null;

                if (pose2d != null) {
                    CommandSwerveDrivetrain.getInstance().addVisionMeasurement(pose2d, est.timestampSeconds);
                    Robot.teleopField.getObject(camera.getName() + " Pose").setPose(pose2d);
                }
            });

        }
    }

    }


