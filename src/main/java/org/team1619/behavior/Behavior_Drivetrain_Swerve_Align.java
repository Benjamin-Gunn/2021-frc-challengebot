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
import org.uacr.utilities.purepursuit.Path;
import org.uacr.utilities.purepursuit.Point;
import org.uacr.utilities.purepursuit.Pose2d;
import org.uacr.utilities.purepursuit.Vector;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


/**
 * Drives the robot in swerve mode, based on the joystick values.
 */

public class Behavior_Drivetrain_Swerve_Align extends BaseSwerve {

    private static final Logger LOGGER = LogManager.getLogger(Behavior_Drivetrain_Swerve_Align.class);

    private final ClosedLoopController headingController;

    private String stateName;

    private double heading;

    public Behavior_Drivetrain_Swerve_Align(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
        super(inputValues, outputValues, config, robotConfiguration);

        headingController = new ClosedLoopController(robotConfiguration.getString("global_drivetrain_Matthew", "heading_controller"));
        
        stateName = "Unknown";

        heading = 0.0;
    }

    @Override
    public void initialize(String stateName, Config config) {
        LOGGER.debug("Entering state {}", stateName);
        this.stateName = stateName;

        stop();

        heading = config.getDouble("heading", heading);

        headingController.setProfile("align");
        headingController.set(heading);
        headingController.reset();
    }

    @Override
    public void update() {
        Map<String, Double> odometryValues = sharedInputValues.getVector(odometry);

        setModulePowers(new Vector(), headingController.getWithPID(odometryValues.get("heading")));
    }

    @Override
    public void dispose() {
        LOGGER.trace("Leaving state {}", stateName);

        stop();
    }

    @Override
    public boolean isDone() {
        return false;
    }
}