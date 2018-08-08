package adapter;

import com.team254.lib.util.math.RigidTransform2d;

public class PathSegment {

    private RigidTransform2d position;

    public PathSegment(RigidTransform2d position) {
        this.position = position;
    }

    public RigidTransform2d getPosition() {
        return position;
    }

}
