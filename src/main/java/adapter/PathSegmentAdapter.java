package adapter;

import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;
import jaci.pathfinder.Trajectory;

public class PathSegmentAdapter extends PathSegment {

    public PathSegmentAdapter(Trajectory.Segment segment) {
        super(new RigidTransform2d(new Translation2d(segment.x, segment.y), Rotation2d.fromRadians(segment.heading)));
    }

}
