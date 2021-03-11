package org.team1619.behavior;

import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;


/**
 * Drives the robot in swerve mode, based on the joystick values.
 */

public class Behavior_Drivetrain_Swerve_Manual extends BaseSwerve {

    private static final Logger LOGGER = LogManager.getLogger(Behavior_Drivetrain_Swerve_Manual.class);

    private final String dPadUp;
    private final String dPadDown;
    private final String dPadLeft;
    private final String dPadRight;
    private final String rightTrigger;

    private double incrementSpeed;
    private double incrementAngle;
    private double speed;
    private double angle;

    private String stateName;

    public Behavior_Drivetrain_Swerve_Manual(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
        super(inputValues, outputValues, config, robotConfiguration, true);

        dPadUp = robotConfiguration.getString("global_drivetrain_swerve", "dpad_up");
        dPadDown = robotConfiguration.getString("global_drivetrain_swerve", "dpad_down");
        dPadLeft = robotConfiguration.getString("global_drivetrain_swerve", "dpad_left");
        dPadRight = robotConfiguration.getString("global_drivetrain_swerve", "dpad_right");
        rightTrigger = robotConfiguration.getString("global_drivetrain_swerve", "right_trigger");

        stateName = "Unknown";
    }

    @Override
    public void initialize(String stateName, Config config) {
        LOGGER.debug("Entering state {}", stateName);
        this.stateName = stateName;

        incrementSpeed = config.getDouble("increment_speed", 0.0);
        incrementAngle = config.getDouble("increment_angle", 0.0);
        speed = 0.0;
        angle = 0.0;
        stopModules();

    }

    @Override
    public void update() {
        boolean speedUp = sharedInputValues.getBooleanRisingEdge(this.dPadUp);
        boolean speedDown = sharedInputValues.getBooleanRisingEdge(this.dPadDown);
        boolean angleLeft = sharedInputValues.getBooleanRisingEdge(this.dPadLeft);
        boolean angleRight = sharedInputValues.getBooleanRisingEdge(this.dPadRight);
        boolean runButton = sharedInputValues.getBoolean(this.rightTrigger);

        if (speed < -1.0){
            speed = -1.0;
        }
        if (speed > 1.0){
            speed = 1.0;
        }
        if (speedUp) {
            speed += incrementSpeed;
        }
        if (speedDown) {
            speed -= incrementSpeed;
        }

        if (angle > 360.0){
            angle = 0.0;
        }
        if(angle < 0){
            angle = 360.0;
        }
        if (angleLeft){
            angle -= incrementAngle;
        }
        if (angleRight){
            angle += incrementAngle;
        }

        sharedInputValues.setNumeric("ipn_drivetrain_swerve_speed", speed);
        sharedInputValues.setNumeric("ipn-drivetrain_swerve_angle", angle);

        if (runButton) {
            sharedOutputValues.setNumeric("opn_drivetrain_front_right", "percent", speed, "pr_drive");
            sharedOutputValues.setNumeric("opn_drivetrain_front_left", "percent", speed, "pr_drive");
            sharedOutputValues.setNumeric("opn_drivetrain_back_right", "percent", speed, "pr_drive");
            sharedOutputValues.setNumeric("opn_drivetrain_back_left", "percent", speed, "pr_drive" );

            sharedOutputValues.setNumeric("opn_drivetrain_front_right_angle", "absolute_position", angle, "pr_drive");
            sharedOutputValues.setNumeric("opn_drivetrain_front_left_angle", "absolute_position", angle, "pr_drive");
            sharedOutputValues.setNumeric("opn_drivetrain_back_right_angle", "absolute_position", angle, "pr_drive");
            sharedOutputValues.setNumeric("opn_drivetrain_back_left_angle", "absolute_position", angle, "pr_drive" );
        }

    }

    @Override
    public void dispose() {
        LOGGER.trace("Leaving state {}", stateName);

        stopModules();
    }

    @Override
    public boolean isDone() {
        return true;
    }
}