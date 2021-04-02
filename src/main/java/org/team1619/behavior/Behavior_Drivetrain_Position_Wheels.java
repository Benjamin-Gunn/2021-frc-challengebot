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

public class Behavior_Drivetrain_Position_Wheels extends BaseSwerve {

    private static final Logger LOGGER = LogManager.getLogger(Behavior_Drivetrain_Position_Wheels.class);

    private String stateName;
    Timer timeoutTimer = new Timer();
    private boolean isDone;
    private boolean messageSent;
    private double wheelAngle;
    private int timeoutTime;

    public Behavior_Drivetrain_Position_Wheels(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
        super(inputValues, outputValues, config, robotConfiguration, true);
    }

    @Override
    public void initialize(String stateName, Config config) {
        LOGGER.debug("Entering state {}", stateName);
        this.stateName = stateName;
        wheelAngle = config.getDouble("wheel_angle");
        timeoutTime = config.getInt("timeout_time");
        angleOutputNames.forEach(output -> sharedOutputValues.setNumeric(output, "absolute_position", wheelAngle, "pr_drive"));
        messageSent = false;
        timeoutTimer.start(timeoutTime);
        isDone = true;
    }

    @Override
    public void update() {

        for (String output: angleOutputNames){
            Object value = sharedOutputValues.getOutputNumericValue(output).get("value");
            if (Math.abs((Double) value - wheelAngle) > 0.2 ){
                isDone = false;
                break;
            }
        }

        if (timeoutTimer.isDone() && !isDone && !messageSent){
            LOGGER.error("*** WHEELS FAILED TO SET TO " + wheelAngle + " degrees ***");
            messageSent = true;
        }

        if(isDone && !messageSent){
            LOGGER.info("*** WHEELS SET TO " + wheelAngle + " degrees ***");
            messageSent = true;
        }

    }


    @Override
    public void dispose() {
        LOGGER.trace("Leaving state {}", stateName);

    }

    @Override
    public boolean isDone() {
        return isDone || timeoutTimer.isDone();
    }
}