package dynamics;

import com.team254.lib.util.math.Twist2d;
import kinematics.DifferentialDriveKinematics;
import kinematics.Movement;

public class DifferentialDriveDynamics {

    private Feedforward leftFeedforward, rightFeedforward;
    private DifferentialDriveKinematics kinematics;

    public DifferentialDriveDynamics(Feedforward leftFeedforward, Feedforward rightFeedforward, DifferentialDriveKinematics kinematics) {
        this.leftFeedforward = leftFeedforward;
        this.rightFeedforward = rightFeedforward;
        this.kinematics = kinematics;
    }

    public double getLeftOutput(double velocity, double accel) {
        return leftFeedforward.getOutput(velocity, accel);
    }

    public double getRightOutput(double velocity, double accel) {
        return rightFeedforward.getOutput(velocity, accel);
    }

    /**
     *
     * @param velocity A Twist2d representing a constant-curvature linear and angular velocity
     * @return Left and right voltages from a linear and angular velocity
     */
    public Movement getOutputs(Twist2d velocity) {
        Movement velocities = kinematics.inverseKinematics(velocity);
        return new Movement(getLeftOutput(velocities.left, 0.0), getRightOutput(velocities.right, 0.0));
    }

}
