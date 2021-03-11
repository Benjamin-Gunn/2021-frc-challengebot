package org.team1619.behavior;

import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.WebDashboardGraphDataset;
import org.uacr.utilities.YamlConfigParser;
import org.uacr.utilities.closedloopcontroller.ClosedLoopController;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import org.uacr.utilities.purepursuit.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;


/**
 * Drives the robot in swerve mode, based on the joystick values.
 */

public class Behavior_Drivetrain_Swerve_Pure_Pursuit extends BaseSwerve {

    private static final Logger LOGGER = LogManager.getLogger(Behavior_Drivetrain_Swerve_Pure_Pursuit.class);

    private final Map<String, Path> mPaths;

    private String stateName;

    private Path currentPath;
    private Pose2d currentPosition;
    private boolean isFollowing;
    private String headingMode;
    private double targetHeading;
    private String pathName;

    public Behavior_Drivetrain_Swerve_Pure_Pursuit(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
        super(inputValues, outputValues, config, robotConfiguration, false);

        stateName = "Unknown";
        
        mPaths = new HashMap<>();

        YamlConfigParser yamlConfigParser = new YamlConfigParser();
        yamlConfigParser.loadWithFolderName("paths.yaml");

        Map<String, Map<String, Object>> paths = ((Map<String, Map<String, Object>>) yamlConfigParser.getData().get("path"));

        if (paths != null) {
            for (String pathName : paths.keySet()) {
                Config pathConfig = yamlConfigParser.getConfig(pathName);

                ArrayList<Point> points = new ArrayList<>();

                pathConfig.getList("path").forEach((p) -> {
                    List point = (List) p;
                    points.add(new Point(point.get(0) instanceof Integer ? (int) point.get(0) : (double) point.get(0), point.get(1) instanceof Integer ? (int) point.get(1) : (double) point.get(1)));
                });

                Path path = new SplinePath(points);

                setupPathUsingConfig(path, pathConfig, yamlConfigParser);

                path.build();

                mPaths.put(pathName, path);

                if("pt_test".equals(pathName)) {
                    graphPath(pathName, path);
                    graphPathWayPoints(pathName, path);
                    graphVelocityProfile(pathName, path);
                }
            }
        }

        pathName = "Unknown";
        headingMode = "none";
        targetHeading = 0.0;

        isFollowing = true;
    }

    @Override
    public void initialize(String stateName, Config config) {
        LOGGER.debug("Entering state {}", stateName);
        this.stateName = stateName;

        stopModules();

        pathName = config.getString("path_name");
        headingMode = config.getString("heading_mode", "navx");
        targetHeading = config.getDouble("target_heading", 0.0);

        if (!mPaths.containsKey(pathName)) {
            LOGGER.error("Path " + pathName + " doesn't exist");
            currentPath = new Path();
        } else {
            currentPath = mPaths.get(pathName);
        }

        graphPath(stateName, currentPath);

        currentPath.reset();

        isFollowing = true;
    }

    @Override
    public void update() {
        if (!isFollowing) {
            stopModules();

            return;
        }

        Map<String, Double> odometryValues = sharedInputValues.getVector(odometry);

        // Turns the odometry values into a Pose2d to pass to path methods
        currentPosition = new Pose2d(odometryValues.get("x"), odometryValues.get("y"), odometryValues.get("heading"));

        int lookahead = currentPath.getLookAheadPointIndex(currentPosition);
        int closest = currentPath.getClosestPointIndex(currentPosition);

        if (lookahead == -1) {
            stopModules();

            isFollowing = false;
            return;
        }

        // Uses the path object to calculate velocity values
        double velocity = currentPath.getPathPointVelocity(closest, currentPosition);

        setModulePowers(new Vector(currentPath.getPoint(lookahead).subtract(currentPosition)).normalize().scale(velocity), headingMode, targetHeading);
    }

    @Override
    public void dispose() {
        LOGGER.trace("Leaving state {}", stateName);

        stopModules();
    }

    @Override
    public boolean isDone() {
        return !isFollowing;
    }

    private void setupPathUsingConfig(Path path, Config config, YamlConfigParser yamlConfigParser) {
        String modelName = config.getString("model", "none");

        if (!modelName.equals("none")) {
            Config model = yamlConfigParser.getConfig(modelName);

            setupPathUsingConfig(path, model, yamlConfigParser);
        }

        path.setPointSpacing(config.getDouble("spacing", path.getPointSpacing()));
        path.setPathSmoothing(config.getDouble("smoothing", path.getPathSmoothing()));
        path.setTurnSpeed(config.getDouble("turn_speed", path.getTurnSpeed()));
        path.setTrackingErrorSpeed(config.getDouble("tracking_error_speed", path.getTrackingErrorSpeed()));
        path.setMaxAcceleration(config.getDouble("max_acceleration", path.getMaxAcceleration()));
        path.setMaxDeceleration(config.getDouble("max_deceleration", path.getMaxDeceleration()));
        path.setMinSpeed(config.getDouble("min_speed", path.getMinSpeed()));
        path.setMaxSpeed(config.getDouble("max_speed", path.getMaxSpeed()));
        path.setLookAheadDistance(config.getDouble("look_ahead_distance", path.getLookAheadDistance()));
        path.setVelocityLookAheadPoints(config.getInt("velocity_look_ahead_points", path.getVelocityLookAheadPoints()));
    }

    private void graphPath(String name, Path path) {
        WebDashboardGraphDataset pathGraphDataset = new WebDashboardGraphDataset();

        for (PathPoint point : path.getPoints()) {
            pathGraphDataset.addPoint(point.getX(), point.getY());
        }

        sharedInputValues.setVector("gr_" + name, pathGraphDataset);
    }

    private void graphPathWayPoints(String name, Path path) {
        WebDashboardGraphDataset pathGraphDataset = new WebDashboardGraphDataset();

      //  for (Point point : path.getWayPoints()) {//     pathGraphDataset.addPoint(point.getX(), point.getY());
       // }

        sharedInputValues.setVector("gr_" + name + "_waypoints", pathGraphDataset);
    }

    private void graphVelocityProfile(String name, Path path) {
        WebDashboardGraphDataset pathGraphDataset = new WebDashboardGraphDataset();

        for (PathPoint point : path.getPoints()) {
            pathGraphDataset.addPoint(point.getDistance(), point.getVelocity());
        }

        sharedInputValues.setVector("gr_" + name + "_velocity", pathGraphDataset);
    }
}