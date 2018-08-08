package kinematics;

import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Twist2d;

public class DifferentialDriveKinematics {

    private final double kEps = 1E-9;

    public final double WHEELBASE, SCRUB_FACTOR, WHEEL_DIAM;
    public final int TICKS_PER_ROTATION;

    public DifferentialDriveKinematics(double WHEELBASE, double SCRUB_FACTOR, double WHEEL_DIAM, int TICKS_PER_ROTATION) {
        this.WHEELBASE = WHEELBASE;
        this.SCRUB_FACTOR = SCRUB_FACTOR;
        this.WHEEL_DIAM = WHEEL_DIAM;
        this.TICKS_PER_ROTATION = TICKS_PER_ROTATION;
    }

    public Twist2d forwardKinematics(double leftDelta, double rightDelta, double angleDelta) {
        return new Twist2d((leftDelta + rightDelta) / 2, 0, angleDelta);
    }

    public Twist2d forwardKinematics(double leftDelta, double rightDelta, Rotation2d angleDelta) {
        return forwardKinematics(leftDelta, rightDelta, angleDelta.getRadians());
    }

    public Twist2d forwardKinematics(double leftDelta, double rightDelta) {
        double delta_v = rightDelta - leftDelta;
        double delta_rotation = delta_v / (WHEELBASE * SCRUB_FACTOR);
        return forwardKinematics(leftDelta, rightDelta, delta_rotation);
    }

    public Movement inverseKinematics(Twist2d twist) {
        if(Math.abs(twist.dx) < kEps) {
            return new Movement(twist.dx, twist.dx);
        }
        double delta_v = WHEELBASE * twist.dtheta / (2 * SCRUB_FACTOR);
        return new Movement(twist.dx - delta_v, twist.dx + delta_v);
    }

    public double ticksToRotations(int ticks) {
        return ticks / TICKS_PER_ROTATION;
    }

    public double ticksToDistance(int ticks) {
        return ticksToRotations(ticks) * (Math.PI * WHEEL_DIAM);
    }

    public double ticksToVel(int ticks) {
        return ticksToRotations(ticks) * (Math.PI * WHEEL_DIAM) * 10;
    }

}
