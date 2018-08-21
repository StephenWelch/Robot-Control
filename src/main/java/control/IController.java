package control;

import physics.Movement;
import math.geometry.Pose;

public interface IController {

    Movement update(Pose pose, double dt);
    boolean isFinished();
    void reset();

}
