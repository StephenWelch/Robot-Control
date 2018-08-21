package adapter;

import math.geometry.RigidTransform2d;
import math.geometry.Rotation2d;
import math.geometry.Translation2d;
import jaci.pathfinder.Trajectory;

public class PathSegmentAdapter extends PathSegment {

    public PathSegmentAdapter(Trajectory.Segment segment) {
        super(new RigidTransform2d(new Translation2d(segment.x, segment.y), Rotation2d.fromRadians(segment.heading)));
    }

}
