package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toDegrees;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class KalmanFusionLocalizer implements Localizer{

    private final Limelight3A limelight;
    private final Localizer base;

    private final KalmanFilter xFilter;
    private final KalmanFilter yFilter;
    private final KalmanFilter thetaFilter;
    private Twist2d diffTwist;

    public KalmanFusionLocalizer(HardwareMap hardwareMap, Localizer baseLocalizer, Pose2d initialPose) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        base = baseLocalizer;
        base.setPose(initialPose);
        KalmanFilterParameters kfParams = new KalmanFilterParameters(6,1); // todo: tune?
        xFilter = new KalmanFilter(kfParams);
        yFilter = new KalmanFilter(kfParams);
        thetaFilter = new KalmanFilter(kfParams);
        diffTwist = new Twist2d(new Vector2d(0, 0), 0);
    }

    public void setPose(Pose2d pose){
        base.setPose(pose);
        xFilter.reset();
        yFilter.reset();
        thetaFilter.reset();
    }

    public void resetPose() {
        setPose(getPose());
    }

    /**
     * Returns the current pose estimate.
     * NOTE: Does not update the pose estimate;
     * you must call update() to update the pose estimate.
     * @return the Localizer's current pose
     */
    public Pose2d getPose(){
        Pose2d pose = base.getPose();
        return pose.plus(diffTwist);
    }

    /**
     * Updates the Localizer's pose estimate.
     * @return the Localizer's current velocity estimate
     */
    public PoseVelocity2d update(){
        //update the base localizer
        PoseVelocity2d baseUpdate = base.update();

        //get the camera
        limelight.updateRobotOrientation(toDegrees(base.getPose().heading.toDouble()));
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid()) {
            // have a camera pose
            Pose3D botPose_mt2 = result.getBotpose_MT2();

            if (botPose_mt2 != null) {
                // good bot pose
                double x = botPose_mt2.getPosition().x;
                double y = botPose_mt2.getPosition().y;
                double theta = botPose_mt2.getOrientation().getYaw();
                Pose2d botPose2d = new Pose2d(x, y, theta);

                // get difference between vision bot pose and odo bot pose
                Twist2d diff = base.getPose().minus(botPose2d);

                //kalman filter the difference between the vision and odometry
                xFilter.update(diff.line.x,0);
                yFilter.update(diff.line.y,0);
                thetaFilter.update(diff.angle, 0);
                diffTwist = new Twist2d(new Vector2d(xFilter.getState(), yFilter.getState()), thetaFilter.getState());
            }
        }
        return baseUpdate;
    }
}
