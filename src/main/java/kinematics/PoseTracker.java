package kinematics;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Twist2d;

public class PoseTracker {

    private DifferentialDriveKinematics driveKinematics;

    //private List<Pose> poseBuffer = new ArrayList<>(1000);

    private InterpolatingTreeMap<InterpolatingDouble, Pose> poseBuffer = new InterpolatingTreeMap<>();

    private double lastLeftAbsolutePosition = 0.0, lastRightAbsolutePosition = 0.0, lastAbsoluteAngle = 0.0;

    public PoseTracker(DifferentialDriveKinematics driveKinematics) {
        poseBuffer.put(new InterpolatingDouble(0.0), new Pose());
        this.driveKinematics = driveKinematics;
    }

    /**
     * Updates the Pose buffer with a Pose derived from absolute measurements.
     * @param leftAbsolutePosition
     * @param rightAbsolutePosition
     * @param absoluteAngle
     * @param leftVelocity
     * @param rightVelocity
     * @param angularVelocity
     * @param timestamp
     */
    public void update(double leftAbsolutePosition, double rightAbsolutePosition, double absoluteAngle, double leftVelocity, double rightVelocity, double angularVelocity, double timestamp) {
        double leftDelta = leftAbsolutePosition - lastLeftAbsolutePosition;
        double rightDelta = rightAbsolutePosition - lastRightAbsolutePosition;
        double angleDelta = absoluteAngle - leftAbsolutePosition;

        Twist2d distanceTraveled = driveKinematics.forwardKinematics(leftDelta, rightDelta, angleDelta);
        Twist2d velocity = driveKinematics.forwardKinematics(leftVelocity, rightVelocity, angularVelocity);

        lastLeftAbsolutePosition = leftAbsolutePosition;
        lastRightAbsolutePosition = rightAbsolutePosition;
        lastAbsoluteAngle = absoluteAngle;

        poseBuffer.put(new InterpolatingDouble(timestamp), getLatest().update(distanceTraveled, velocity));
    }

    /**
     * Updates the Pose buffer with a Pose with estimated angular values.
     * @param leftAbsolutePosition
     * @param rightAbsolutePosition
     * @param leftVelocity
     * @param rightVelocity
     * @param timestamp
     */
    public void update(double leftAbsolutePosition, double rightAbsolutePosition, double leftVelocity, double rightVelocity, double timestamp) {
        double leftDelta = leftAbsolutePosition - lastLeftAbsolutePosition;
        double rightDelta = rightAbsolutePosition - lastRightAbsolutePosition;

        Twist2d distanceTraveled = driveKinematics.forwardKinematics(leftDelta, rightDelta);
        Twist2d velocity = driveKinematics.forwardKinematics(leftVelocity, rightVelocity);

        lastLeftAbsolutePosition = leftAbsolutePosition;
        lastRightAbsolutePosition = rightAbsolutePosition;

        poseBuffer.put(new InterpolatingDouble(timestamp), getLatest().update(distanceTraveled, velocity));
    }

    /**
     * Reset the Pose buffer to a new Pose.
     * @param pose
     * @param timestamp
     */
    public void reset(Pose pose, double timestamp) {
        poseBuffer.put(new InterpolatingDouble(timestamp), pose);
    }

    /**
     * Convenience method to allow resetting to a certain field location.
     * @param position
     * @param timestamp
     */
    public void reset(RigidTransform2d position, double timestamp) {
        reset(new Pose(position, new RigidTransform2d()), timestamp);
    }

    /**
     * Convenience method to reset to origin.
     * @param timestamp
     */
    public void reset(double timestamp) {
        reset(new Pose(), timestamp);
    }

    public Pose getLatest() {
        return poseBuffer.lastEntry().getValue();
    }

    public double getLatestTime() {
        return poseBuffer.lastEntry().getKey().value;
    }

    public Pose get(double timestamp) {
        return poseBuffer.getInterpolated(new InterpolatingDouble(timestamp));
    }

}
