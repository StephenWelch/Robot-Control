package kinematics;

public class Movement {
    public final double left, right;

    public Movement(double left, double right) {
        this.left = left;
        this.right = right;
    }

    public Movement add(Movement other) {
        return new Movement(left + other.left, right + other.right);
    }

}
