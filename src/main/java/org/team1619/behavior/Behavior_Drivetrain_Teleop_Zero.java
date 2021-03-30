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
    private final List<Double> angleValues = new ArrayList<>();

    public Behavior_Drivetrain_Teleop_Zero(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
        super(inputValues, outputValues, config, robotConfiguration, true);


    }

    @Override
    public void initialize(String stateName, Config config) {
        LOGGER.debug("Entering state {}", stateName);
        this.stateName = stateName;
      angleOutputNames.forEach(output -> sharedOutputValues.setNumeric(output,"absolute_position", 0.0, "pr_drive"));
      timeoutTimer.start(2000);
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
            if(timeoutTimer.isDone() && !(Math.abs(value) < 0.5)){
                LOGGER.debug("***TELEOP ZERO TIMEOUT***");
            }
            done = Math.abs(value) < 0.5 || timeoutTimer.isDone();
        }

        if(done){
            LOGGER.debug("***WHEELS STRAIGHT***");
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