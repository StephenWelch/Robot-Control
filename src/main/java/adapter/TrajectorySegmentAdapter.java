package adapter;

import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;
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
