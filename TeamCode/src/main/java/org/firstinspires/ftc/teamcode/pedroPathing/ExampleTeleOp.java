package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Math.toDegrees;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.function.Supplier;
@Configurable
@TeleOp
public class ExampleTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private Limelight3A limelight; //any camera here

    enum allianceColor {
        RED,
        BLUE
    }

    private final KalmanFilterParameters kfParams = new KalmanFilterParameters(6,1); // todo: tune?

    private final KalmanFilter xFilter =  new KalmanFilter(kfParams);
    private final KalmanFilter yFilter = new KalmanFilter(kfParams);
    private final KalmanFilter thetaFilter = new KalmanFilter(kfParams);

    // TODO: set this at init
    allianceColor alliance = allianceColor.RED;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)

        limelight.start();
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        // snap to goal
        boolean snapToGoal = gamepad1.left_bumper;
        Pose goalPose;
        //Call this once per loop
        follower.update();
        telemetryM.update();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );

            if (snapToGoal) {
                // determine field goal position
                switch (alliance) {
                    case BLUE:
                        goalPose = new Pose(4, 140);
                        break;
                    case RED:
                    default:
                        goalPose = new Pose(140, 140);
                        break;
                }
                // determine robot pose
                Pose robotPose = follower.getPose();
                // calculate heading to goal
                double headingToGoal = Math.atan2((goalPose.getY() - robotPose.getY()), (goalPose.getX() - robotPose.getX()));
                // apply controls to teleop drive w/ calculated heading (field relative)
                // turn is field relative
                follower.turnTo(headingToGoal);
            }
        }
        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }
        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        //This uses the aprilTag to relocalize your robot
        // poseTracker will apply the offset below to pose when getPose is called
        PoseTracker poseTracker = follower.getPoseTracker();
        Pose poseOffset = getRobotOffsetFromCamera();
        poseTracker.setXOffset(poseOffset.getX());
        poseTracker.setYOffset(poseOffset.getY());
        poseTracker.setHeadingOffset(poseOffset.getHeading());
    }
    private Pose getRobotOffsetFromCamera() {
        //get the camera
        limelight.updateRobotOrientation(toDegrees(follower.getPoseTracker().getLocalizer().getIMUHeading()));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // have a camera pose
            Pose3D botPose_mt2 = result.getBotpose_MT2();

            if (botPose_mt2 != null) {
                // good bot pose
                double x = botPose_mt2.getPosition().x;
                double y = botPose_mt2.getPosition().y;
                double theta = botPose_mt2.getOrientation().getYaw();
                Pose botPose2d = new Pose(x, y, theta);

                // get difference between vision bot pose and odo bot pose
                Pose diff = follower.getPoseTracker().getRawPose().minus(botPose2d);

                //kalman filter the difference between the vision and odometry
                xFilter.update(diff.getX(), 0);
                yFilter.update(diff.getX(), 0);
                thetaFilter.update(diff.getHeading(), 0);
            }
        }
        return new Pose(xFilter.getState(), yFilter.getState(), thetaFilter.getState());
    }
}