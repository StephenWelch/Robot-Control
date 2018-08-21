package adapter;

import math.geometry.RigidTransform2d;
import math.geometry.Rotation2d;
import math.geometry.Translation2d;
import jaci.pathfinder.Trajectory;

public class TrajectorySegmentAdapter extends TrajectorySegment {


    public TrajectorySegmentAdapter(Trajectory.Segment segment) {

        super(new RigidTransform2d(new Translation2d(segment.x, segment.y), Rotation2d.fromRadians(segment.heading)),
                new RigidTransform2d(segment.velocity, segment.heading),
                new RigidTransform2d(segment.acceleration, segment.heading),
                new RigidTransform2d(segment.jerk, segment.heading),
                segment.dt);
    }

}
