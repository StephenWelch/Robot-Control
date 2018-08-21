package physics;

import math.Util;

public class Movement {
    public final double left, right;

    public Movement(double left, double right) {
        this.left = left;
        this.right = right;
    }

    public Movement add(Movement other) {
        return new Movement(left + other.left, right + other.right);
    }

    public String toString() {
        return String.format("Left: %s\tRight:%s", left, right);
    }

    public static Movement clamp(Movement movement, double leftMagnitude, double rightMagnitude) {
        double left, right = 0;

        left = Util.limit(movement.left, leftMagnitude);
        right = Util.limit(movement.right, rightMagnitude);

        return new Movement(left, right);
    }

}
