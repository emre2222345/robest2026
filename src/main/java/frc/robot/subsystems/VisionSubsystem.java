// =============================================================================
// VisionSubsystem.java
// =============================================================================
// Subsystem responsible for vision-based robot pose estimation using a
// Limelight 3A camera with AprilTag detection.
//
// COORDINATE SYSTEM:
//   - WPILib uses a field-relative coordinate system with the origin at the
//     bottom-left corner of the field from the perspective of the blue alliance.
//   - "botpose_wpiblue" returns pose in this coordinate system, so no
//     alliance-based coordinate flip is needed.
//   - X+  = toward red alliance wall
//   - Y+  = toward the left side of the field (from blue alliance perspective)
//   - Rotation follows WPILib convention: counterclockwise positive.
//
// POSE UNITS:
//   - Limelight 3A AprilTag pipelines return translation in METERS and
//     rotation in DEGREES. This subsystem assumes meters. If you use a
//     custom pipeline that returns inches, you must convert before passing
//     to Pose2d or all downstream math will be wrong.
//
// LATENCY & MOTION COMPENSATION:
//   - This subsystem corrects the timestamp for pipeline latency so that
//     callers receive the approximate FPGA time the image was captured.
//   - It does NOT correct the pose itself for robot motion during latency.
//   - Motion compensation is intentionally delegated to WPILib's
//     PoseEstimator (SwerveDrivePoseEstimator / DifferentialDrivePoseEstimator),
//     which applies it internally via Kalman filtering when you pass the
//     latency-corrected timestamp. Do NOT apply motion correction here —
//     doing so would double-correct when used with PoseEstimator.
//
// ASSUMPTIONS:
//   - Limelight is configured and streaming over NetworkTables (default port).
//   - The Limelight pipeline is set to detect AprilTags.
//   - "botpose_wpiblue" returns an 8-element extended array (see BOTPOSE layout).
//   - Robot pose is updated ONLY from vision; no wheel odometry or gyro is used.
//   - Simulation: NetworkTables returns default empty arrays, which are caught
//     by validity gates and safely ignored. Use injectSimPose() for sim testing.
// =============================================================================

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * VisionSubsystem
 *
 * <p>Reads AprilTag-based robot pose from a Limelight 3A via NetworkTables.
 * Pose is expressed in the WPILib blue-alliance field coordinate system using
 * the "botpose_wpiblue" key. Updates are filtered for validity before being
 * stored. No drivetrain, gyro, or wheel odometry is used.
 *
 * <p>Simulation: In sim, NetworkTables returns default (empty) arrays, which
 * are caught by validity checks and safely discarded. Call
 * {@link #injectSimPose(Pose2d)} to feed a synthetic pose for testing.
 */
public class VisionSubsystem extends SubsystemBase {

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------

    /**
     * "botpose_wpiblue" extended array layout (8 elements):
     *   [0] = X (meters)
     *   [1] = Y (meters)
     *   [2] = Z (meters)         -- ignored for 2D pose
     *   [3] = Roll  (degrees)    -- ignored for 2D pose
     *   [4] = Pitch (degrees)    -- ignored for 2D pose
     *   [5] = Yaw   (degrees)    -- used as Rotation2d heading
     *   [6] = Total latency (ms) -- pipeline latency + capture latency
     *   [7] = Tag count          -- number of tags contributing to the pose
     */
    private static final String BOTPOSE_KEY = "botpose_wpiblue";

    /**
     * Minimum array length required to extract both pose AND latency.
     * The extended botpose_wpiblue array has 8 elements; we require at least
     * 7 (indices 0-6) to read pose + latency. Tag count (index 7) is read
     * only when the full 8-element array is present.
     */
    private static final int BOTPOSE_MIN_LENGTH = 7;

    /**
     * Full length of the extended botpose_wpiblue array, including tag count.
     * Referenced by the tag-count gate to avoid an index-out-of-bounds guard.
     */
    private static final int BOTPOSE_EXTENDED_LENGTH = 8;

    /**
     * Minimum number of AprilTags that must contribute to a pose measurement
     * for it to be accepted. This is the primary validity gate — a tagCount
     * of 0 means the Limelight has no real target, regardless of what values
     * the pose array contains.
     *
     * Set to 1 to accept single-tag poses.
     * Set to 2 for improved accuracy (rejects noisier single-tag estimates).
     */
    private static final int MIN_TAG_COUNT = 1;

    /**
     * Maximum age (in seconds) of the last accepted pose before hasPose()
     * returns false. Prevents a stale pose from appearing valid after vision
     * is temporarily lost (e.g., tags go out of view mid-match).
     */
    private static final double POSE_STALENESS_THRESHOLD_SEC = 0.5;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    /** NetworkTable for this Limelight instance. */
    private final NetworkTable limelightTable;

    /**
     * The most recently accepted, validated robot pose.
     * Initialized to the field origin. Will not change until a valid
     * vision measurement is received.
     */
    private Pose2d currentPose = new Pose2d();

    /**
     * FPGA timestamp (seconds) of the last accepted pose, latency-corrected
     * to approximate the actual image capture time.
     *
     * Used by PoseEstimator callers via getLastPoseTimestamp().
     * Initialized to 0; callers should check hasPose() before using.
     */
    private double lastPoseTimestamp = 0.0;

    /** Field2d widget for SmartDashboard / Elastic visualization. */
    private final Field2d field2d = new Field2d();

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * Constructs the VisionSubsystem using the default Limelight name.
     * Equivalent to {@code new VisionSubsystem("limelight")}.
     */
    public VisionSubsystem() {
        this("limelight");
    }

    /**
     * Constructs the VisionSubsystem with a specific Limelight NetworkTables name.
     *
     * <p>Use this constructor when:
     * <ul>
     *   <li>Your Limelight is renamed in the web UI (e.g., "limelight-front")</li>
     *   <li>You have multiple Limelights and want separate subsystem instances</li>
     *   <li>You want to inject a mock table name in unit/simulation tests</li>
     * </ul>
     *
     * @param limelightName The NetworkTables table name for the Limelight,
     *                      as configured in the Limelight web interface.
     */
    public VisionSubsystem(String limelightName) {
        limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
        SmartDashboard.putData("Vision/Field", field2d);
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    /**
     * Called once per scheduler cycle (~20 ms).
     * Polls the Limelight for a new pose estimate, validates it, and stores
     * it if valid.
     */
    @Override
    public void periodic() {
        updatePoseFromVision();
        publishTelemetry();
    }

    // -------------------------------------------------------------------------
    // Core Vision Logic
    // -------------------------------------------------------------------------

    /**
     * Reads "botpose_wpiblue" from NetworkTables, validates the result, and
     * updates {@code currentPose} with a latency-corrected FPGA timestamp.
     *
     * <p>Validation gates (in order):
     * <ol>
     *   <li>Array length — must have at least {@value #BOTPOSE_MIN_LENGTH} elements.</li>
     *   <li>Tag count — must have at least {@value #MIN_TAG_COUNT} contributing tag(s).
     *       This is the primary gate; a zero tag count means no real target.</li>
     *   <li>Zero-pose fallback — secondary defense against a misconfigured pipeline
     *       returning a non-zero tag count with a zeroed pose array.</li>
     * </ol>
     */
    private void updatePoseFromVision() {
        double[] botpose = limelightTable
                .getEntry(BOTPOSE_KEY)
                .getDoubleArray(new double[0]);

        // Gate 1: Array length.
        // Must have at least 7 elements to read pose (indices 0-5) + latency (index 6).
        if (botpose.length < BOTPOSE_MIN_LENGTH) {
            return;
        }

        // Gate 2: Tag count (primary validity check).
        // Index 7 is only present in the full 8-element extended array.
        // If present and below threshold, reject immediately — no real target.
        if (botpose.length >= BOTPOSE_EXTENDED_LENGTH) {
            int tagCount = (int) botpose[7];
            if (tagCount < MIN_TAG_COUNT) {
                return;
            }
        }

        // Gate 3: Zero-pose fallback (secondary defense).
        // A pose of exactly (0, 0, yaw=0) most likely indicates a null response
        // from the Limelight. A robot physically cannot be at the exact field-origin
        // corner with 0 heading, so this is a safe heuristic. Tag-count (gate 2)
        // is the stronger check; this catches edge cases where tag count is missing.
        double x   = botpose[0];
        double y   = botpose[1];
        double yaw = botpose[5];

        if (x == 0.0 && y == 0.0 && yaw == 0.0) {
            return;
        }

        // Build Pose2d.
        // Limelight yaw is in degrees; Rotation2d.fromDegrees() converts correctly.
        // Translation is in meters (Limelight 3A AprilTag pipeline default).
        Pose2d visionPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));

        // Latency-corrected FPGA timestamp.
        // botpose[6] = total pipeline latency in milliseconds.
        // We convert to seconds and subtract from the current FPGA clock to get
        // the approximate time the image frame was captured.
        // NOTE: Do NOT apply motion correction to the pose here. PoseEstimator
        // handles motion compensation internally using this timestamp.
        double latencySeconds = botpose[6] / 1000.0;
        double imageTimestamp = Timer.getFPGATimestamp() - latencySeconds;

        currentPose       = visionPose;
        lastPoseTimestamp = imageTimestamp;
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    /**
     * Publishes current pose and vision health metrics to SmartDashboard / Elastic.
     */
    private void publishTelemetry() {
        field2d.setRobotPose(currentPose);

        SmartDashboard.putNumber("Vision/Pose X (m)",      currentPose.getX());
        SmartDashboard.putNumber("Vision/Pose Y (m)",      currentPose.getY());
        SmartDashboard.putNumber("Vision/Heading (deg)",   currentPose.getRotation().getDegrees());
        SmartDashboard.putBoolean("Vision/Has Valid Pose", hasPose());
        SmartDashboard.putNumber("Vision/Last Timestamp",  lastPoseTimestamp);
    }

    // -------------------------------------------------------------------------
    // Simulation Support
    // -------------------------------------------------------------------------

    /**
     * Injects a synthetic pose into the Limelight NetworkTable for simulation
     * and unit testing. Has no effect on a real robot.
     *
     * <p>This writes a fully valid 8-element botpose_wpiblue array directly
     * into NetworkTables so the normal {@link #updatePoseFromVision()} path
     * picks it up without any special-case sim branching in periodic().
     *
     * <p>Example usage in a sim test or RobotContainer sim init:
     * <pre>
     *   visionSubsystem.injectSimPose(new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(45)));
     * </pre>
     *
     * @param pose The {@link Pose2d} to inject as the simulated robot pose.
     *             Translation must be in meters; rotation in the WPILib convention.
     */
    public void injectSimPose(Pose2d pose) {
        if (!RobotBase.isSimulation()) return;

        // Build a valid 8-element botpose_wpiblue array.
        // Latency is 0 ms (no simulated delay) and tag count is 1 so all gates pass.
        double[] simBotpose = new double[] {
            pose.getX(),                        // [0] X (meters)
            pose.getY(),                        // [1] Y (meters)
            0.0,                                // [2] Z (ignored)
            0.0,                                // [3] Roll (ignored)
            0.0,                                // [4] Pitch (ignored)
            pose.getRotation().getDegrees(),    // [5] Yaw (degrees)
            0.0,                                // [6] Latency ms — 0 for sim
            1.0                                 // [7] Tag count — 1 to pass gate
        };

        limelightTable.getEntry(BOTPOSE_KEY).setDoubleArray(simBotpose);
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /**
     * Returns the most recently accepted vision-based robot pose.
     *
     * <p>Pose is in the WPILib blue-alliance field coordinate system:
     * origin at the bottom-left corner of the field (blue alliance side),
     * X+ toward the red alliance wall, Y+ toward the left sideline.
     * Translation is in meters.
     *
     * <p>If no valid measurement has been received, or the last measurement
     * is stale (older than {@value #POSE_STALENESS_THRESHOLD_SEC} s), this
     * still returns the last known pose — but {@link #hasPose()} will return
     * false. Callers that must not act on a stale pose should check hasPose()
     * before using this value.
     *
     * @return The latest valid {@link Pose2d}, or the field origin if none received.
     */
    public Pose2d getCurrentPose() {
        return currentPose;
    }

    /**
     * Returns {@code true} if a valid vision measurement has been received
     * recently (within {@value #POSE_STALENESS_THRESHOLD_SEC} seconds).
     *
     * <p>Unlike a one-way latch, this resets to {@code false} when vision is
     * lost (e.g., tags go out of view), preventing stale poses from silently
     * appearing valid to callers like an auto path follower.
     *
     * @return {@code true} if the pose is fresh and trustworthy.
     */
    public boolean hasPose() {
        if (lastPoseTimestamp == 0.0) return false;
        return (Timer.getFPGATimestamp() - lastPoseTimestamp) < POSE_STALENESS_THRESHOLD_SEC;
    }

    /**
     * Returns the latency-corrected FPGA timestamp (seconds) at which the last
     * accepted pose measurement was captured.
     *
     * <p>Pass this directly to {@code SwerveDrivePoseEstimator.addVisionMeasurement()}
     * or {@code DifferentialDrivePoseEstimator.addVisionMeasurement()} — WPILib's
     * Kalman filter uses this timestamp to retroactively apply the measurement at
     * the correct point in the odometry history, handling motion compensation
     * automatically.
     *
     * @return Latency-corrected image capture timestamp in seconds.
     *         Returns 0.0 if no valid pose has been received yet.
     */
    public double getLastPoseTimestamp() {
        return lastPoseTimestamp;
    }

    /**
     * Returns the {@link Field2d} widget used by this subsystem.
     *
     * <p>Expose this to other subsystems or RobotContainer so they can draw
     * additional objects on the same field widget — for example, trajectory
     * previews, vision target markers, or waypoints — without creating a
     * duplicate field widget on SmartDashboard.
     *
     * @return The shared {@link Field2d} instance.
     */
    public Field2d getField2d() {
        return field2d;
    }

    /**
     * Resets the stored pose to the given value and clears the pose timestamp.
     *
     * <p>Use this when you have a known reliable ground truth — for example,
     * at the start of autonomous when the robot is placed at a known position,
     * or after vision is regained following a long outage and you want to
     * force-seed the estimator.
     *
     * <p>After calling this, {@link #hasPose()} will return {@code false} until
     * the next valid vision measurement is accepted, since the injected pose
     * carries no vision timestamp.
     *
     * @param pose The {@link Pose2d} to seed as the current robot pose.
     */
    public void resetPose(Pose2d pose) {
        currentPose       = pose;
        lastPoseTimestamp = 0.0; // Clears hasPose() until next real measurement.
    }

    // =========================================================================
    // FUTURE INTEGRATION — SwerveDrivePoseEstimator
    // =========================================================================
    //
    // When you add a drivetrain, wire this subsystem into WPILib's estimator:
    //
    //   SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    //       kinematics, gyroAngle, modulePositions, initialPose,
    //       stateStdDevs,            // trust in wheel odometry
    //       visionMeasurementStdDevs // trust in vision
    //   );
    //
    //   // In DriveSubsystem.periodic(), after vision subsystem runs:
    //   if (visionSubsystem.hasPose()) {
    //       poseEstimator.addVisionMeasurement(
    //           visionSubsystem.getCurrentPose(),
    //           visionSubsystem.getLastPoseTimestamp()  // latency-corrected
    //       );
    //   }
    //
    //   // Draw the estimator's fused pose on the same field widget:
    //   visionSubsystem.getField2d()
    //       .getObject("Estimated Pose")
    //       .setPose(poseEstimator.getEstimatedPosition());
    //
    // No changes to VisionSubsystem are needed for this integration.
    // =========================================================================
}