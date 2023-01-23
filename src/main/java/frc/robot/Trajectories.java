package frc.robot;

import static frc.robot.Constants.PathConstants;

import java.util.HashMap;
import com.pathplanner.lib.*;

public class Trajectories {
    private HashMap<String, PathPlannerTrajectory> paths;

    public Trajectories() {
        paths = new HashMap<>();
        // Load all path files.
        for (String pathName : PathConstants.names) {
            paths.put(pathName, PathPlanner.loadPath(pathName, new PathConstraints(2, 1)));
        }
    }

    public PathPlannerTrajectory getTrajectory(String pathName) {
        return paths.get(pathName);
    }
}
