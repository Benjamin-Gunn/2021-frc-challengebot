package org.team1619.behavior;

import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.Timer;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.*;
import java.util.stream.Stream;

/**
 * Zeros the swerve modules
 */

public class Behavior_Drivetrain_Teleop_Zero extends BaseSwerve {

    private static final Logger LOGGER = LogManager.getLogger(Behavior_Drivetrain_Zero.class);

    private int timeoutTime;
    private double zeroingThreshold;
    private String stateName;
    private boolean done = false;

    public Behavior_Drivetrain_Teleop_Zero(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
        super(inputValues, outputValues, config, robotConfiguration, true);


    }

    @Override
    public void initialize(String stateName, Config config) {
        LOGGER.debug("Entering state {}", stateName);
        this.stateName = stateName;
      angleOutputNames.stream().forEach(output -> sharedOutputValues.setNumeric(output,"absolute_position", 0.0, "pr_drive"));
    }

    @Override
    public void update() {
        for (String output : angleOutputNames) {
            if (output.equals(0.0)) {
                done = true;
            } else {
                done = false;
            }
        }
    }


    @Override
    public void dispose() {
        LOGGER.trace("Leaving state {}", stateName);

    }

    @Override
    public boolean isDone() {
        return done ? true : false;
    }
}