package control;

import kinematics.Movement;
import kinematics.Pose;

public interface IController {

    Movement update(Pose pose, double dt);
    boolean isFinished();

}
