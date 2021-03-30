package org.team1619.behavior;

import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.Timer;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.*;

/**
 * Zeros the swerve modules
 */

public class Behavior_Drivetrain_Teleop_Zero extends BaseSwerve {

    private static final Logger LOGGER = LogManager.getLogger(Behavior_Drivetrain_Zero.class);

    private String stateName;
    Timer timeoutTimer = new Timer();
    private boolean done = false;
    private double motorPosition;
    private int timeoutTime;
    private final List<Double> angleValues = new ArrayList<>();

    public Behavior_Drivetrain_Teleop_Zero(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
        super(inputValues, outputValues, config, robotConfiguration, true);


    }

    @Override
    public void initialize(String stateName, Config config) {
        LOGGER.debug("Entering state {}", stateName);
        this.stateName = stateName;
        motorPosition = config.getDouble("motor_position");
        timeoutTime = config.getInt("timeout_time");

      angleOutputNames.forEach(output -> sharedOutputValues.setNumeric(output,"absolute_position", motorPosition, "pr_drive"));
      timeoutTimer.start(timeoutTime);
    }

    @Override
    public void update() {

        angleOutputNames.forEach(output -> {
            Object value = sharedOutputValues.getOutputNumericValue(output).get("value");
            if (value instanceof Double){
                angleValues.add((Double) value);
            }
        });

        for (Double value : angleValues){
            if(timeoutTimer.isDone() && !(Math.abs(value) < 0.2)){
                LOGGER.error("***WHEELS FAILED TO SET TO " + motorPosition + "***");
            }
            done = Math.abs(value) < 0.2 || timeoutTimer.isDone();
        }

        if(done){
            LOGGER.debug("***WHEELS SET TO " + motorPosition + "***");
        }
    }


    @Override
    public void dispose() {
        LOGGER.trace("Leaving state {}", stateName);

    }

    @Override
    public boolean isDone() {
        return done;
    }
}