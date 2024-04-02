// CREDIT: YAGSL

package org.usfirst.frc4904.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class SwerveSubsystem extends SubsystemBase {
    /**
     * Angle conversion factor.
     */
    private final double angleConversionFactor;

    /**
     * Drive conversion factor.
     */
    private final double driveConversionFactor;

    /**
     * Swerve drive object.
     */
    public final SwerveDrive swerveDrive;

    /**
     * Maximum speed of the robot in meters per second, used to limit acceleration.
     */
    private final double maximumSpeed; //theres a merge conflict here but its actually fine and i need to manually resove it so im typing a comment

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public SwerveSubsystem(File directory, double angleFactor, double driveFactor, double maxSpeed) throws IOException {
        // angle conversion factor is 360 / (gear ratio * encoder resolution)
        // in this case the gear ratio is 12.8 motor revolutions per wheel rotation.
        // the encoder resolution per motor revolution is 1 per motor revolution.
        this.angleConversionFactor = angleFactor;
        this.driveConversionFactor = driveFactor;
        this.maximumSpeed = maxSpeed;
        // motor conversion factor is (pi * wheel diameter in meters) / (gear ratio * encoder resolution).
        // in this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
        // the gear ratio is 6.75 motor revolutions per wheel rotation.
        // the encoder resolution per motor revolution is 1 per motor revolution.
        System.out.println("\"conversionFactor\": {");
        System.out.println("\t\"angle\": " + angleConversionFactor + ",");
        System.out.println("\t\"drive\": " + driveConversionFactor);
        System.out.println("}");

        // configure the telemetry before creating the swerve drive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        // heading correction should only be used while controlling the robot via angle.
        swerveDrive.setHeadingCorrection(false);

        setupPathPlanner();
    }
    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            // robot pose supplier
            this::getPose,

            // method to reset odometry
            this::resetOdometry,

            // chassis speed supplier, must be robot relative
            this::getRobotVelocity,

            // method that will drive robot given robot relative chassis speeds
            this::setChassisSpeeds,

            // holonomic path followers configuration
            new HolonomicPathFollowerConfig(
                // translation pid constants
                new PIDConstants(5.0, 0.0, 0.0),

                // rotation pid constants
                new PIDConstants(
                    swerveDrive.swerveController.config.headingPIDF.p,
                    swerveDrive.swerveController.config.headingPIDF.i,
                    swerveDrive.swerveController.config.headingPIDF.d
                ),

                // max module speed, in m/s
                4.5,

                // drive base radius in meters. distance from robot center to furthest module.
                swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),

                // default path replanning config
                new ReplanningConfig()
            ),

            () -> {
                // controls when path is mirrored for red alliance
                // TODO: Blue alliance should be red alliance but fixed it badly pls change
                var alliance = DriverStation.getAlliance();
                // return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue;
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;    
            },

            // reference to this subsystem
            this
        );
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName       PathPlanner path name.
     * @param setOdomToStart Set the odometry position to the start of the path.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        if (setOdomToStart) {
            resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
        }

        // create path following command using autobuilder
        return AutoBuilder.followPath(path);
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        // create pathfinding constraints
        PathConstraints constraints = new PathConstraints(
            swerveDrive.getMaximumVelocity(),
            4.0,
            swerveDrive.getMaximumAngularVelocity(),
            Units.degreesToRadians(720)
        );

        // build pathfinding commands using configuring autobuilder
        return AutoBuilder.pathfindToPose(pose, constraints, 0.0, 0.0);
    }

    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother controls.
     * @param translationY Translation in the Y direction. Cubed for smoother controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(
        DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY
    ) {
        return run(() -> {
            // smooth control
            double xInput = Math.pow(translationX.getAsDouble(), 3);
            double yInput = Math.pow(translationY.getAsDouble(), 3);

            // make robot move
            driveFieldOriented(
                swerveDrive.swerveController.getTargetSpeeds(
                    xInput,
                    yInput,
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumVelocity()
                )
            );
        });
    }

    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param rotation     Rotation as a value between [-1, 1] converted to radians.
     * @return Drive command.
     */
    public Command simDriveCommand(
        DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation
    ) {
        return run(() -> {
            // make robot move
            driveFieldOriented(
                swerveDrive.swerveController.getTargetSpeeds(
                    translationX.getAsDouble(),
                    translationY.getAsDouble(),
                    rotation.getAsDouble() * Math.PI,
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumVelocity()
                )
            );
        });
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
     * @return Drive command.
     */
    public Command driveCommand(
        DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX
    ) {
        return run(() -> {
            // make robot move
            swerveDrive.drive(
                new Translation2d(Math.pow(translationX.getAsDouble(), 1) * swerveDrive.getMaximumVelocity(),
                Math.pow(translationY.getAsDouble(), 1) * swerveDrive.getMaximumVelocity()),
                Math.pow(angularRotationX.getAsDouble(), 1) * swerveDrive.getMaximumAngularVelocity(),
                true,
                false
            );
            

            // set led to robot in motion
            RobotMap.Component.led.setRobotInMotion(translationX.getAsDouble(), translationY.getAsDouble(), angularRotationX.getAsDouble());
        });
    }

    /**
     * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
     * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
     *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
     *                      (field North) and positive y is towards the left wall when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(
            translation, rotation, fieldRelative, false
        );
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(
        double xInput, double yInput, double headingX, double headingY
    ) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);

        return swerveDrive.swerveController.getTargetSpeeds(
            xInput,
            yInput,
            headingX,
            headingY,
            getHeading().getRadians(),
            maximumSpeed
        );
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
     * 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                angle.getRadians(),
                getHeading().getRadians(),
                maximumSpeed);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    /**
     * Add a fake vision reading for testing purposes.
     */
    public void addFakeVisionReading() {
        swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }


  // public Optional<EstimatedRobotPose> updateVisionEstimate()
  // {
  //   return photonPoseEstimator.update();
  // }

//   /**
//    * @param estimatedRobotPose result from updateVisionEstimate()
//    */
//   public void addVisionMeasurement()
//   {
//     Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();
//     if (estimatedRobotPose.isEmpty()){
//       return;
//     }
//     SmartDashboard.putNumber("vision X", estimatedRobotPose.get().estimatedPose.getX());
//     SmartDashboard.putNumber("vision Y", estimatedRobotPose.get().estimatedPose.getY());
//     SmartDashboard.putNumber("vision Z", estimatedRobotPose.get().estimatedPose.getZ());
//     swerveDrive.addVisionMeasurement(estimatedRobotPose.get().estimatedPose.toPose2d(), estimatedRobotPose.get().timestampSeconds);
//   }

    public void brickMode() {
        swerveDrive.swerveModules[0].setAngle(45);
        swerveDrive.swerveModules[1].setAngle(-45);
        swerveDrive.swerveModules[2].setAngle(-45);
        swerveDrive.swerveModules[3].setAngle(45);
    }
}
