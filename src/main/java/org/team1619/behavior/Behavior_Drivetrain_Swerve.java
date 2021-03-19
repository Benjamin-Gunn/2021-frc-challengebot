package org.team1619.behavior;

import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import org.uacr.utilities.purepursuit.Point;
import org.uacr.utilities.purepursuit.Vector;


/**
 * Drives the robot in swerve mode, based on the joystick values.
 */

public class Behavior_Drivetrain_Swerve extends BaseSwerve {

    private static final Logger LOGGER = LogManager.getLogger(Behavior_Drivetrain_Swerve.class);

    private final String xAxis;
    private final String yAxis;
    private final String rotateAxis;
    private final String fFieldOrientedButton;

    private final String slowModeButton;
    private final double slowModeMaxVelocity;
    private final String cornerModeButton;
    private final double cornerModeMaxVelocity;

    private String stateName;

    private boolean fieldOriented;

    private boolean targetLimelight;

    public Behavior_Drivetrain_Swerve(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
        super(inputValues, outputValues, config, robotConfiguration, true);

        xAxis = robotConfiguration.getString("global_drivetrain_swerve", "swerve_x");
        yAxis = robotConfiguration.getString("global_drivetrain_swerve", "swerve_y");
        rotateAxis = robotConfiguration.getString("global_drivetrain_swerve", "swerve_rotate");
        fFieldOrientedButton = robotConfiguration.getString("global_drivetrain_swerve", "swerve_field_oriented_button");

        slowModeButton = robotConfiguration.getString("global_drivetrain_swerve", "slow_mode_button");
        slowModeMaxVelocity = robotConfiguration.getInt("global_drivetrain_swerve", "slow_mode_max_velocity");
        cornerModeButton = robotConfiguration.getString("global_drivetrain_swerve", "corner_mode_button");
        cornerModeMaxVelocity = robotConfiguration.getInt("global_drivetrain_swerve", "corner_mode_max_velocity");
        stateName = "Unknown";

        fieldOriented = true;
        targetLimelight = false;
    }

    @Override
    public void initialize(String stateName, Config config) {
        LOGGER.debug("Entering state {}", stateName);
        this.stateName = stateName;

        stopModules();

        targetLimelight = config.getBoolean("target_limelight", false);

    }

    @Override
    public void update() {

        if (sharedInputValues.getBooleanRisingEdge(fFieldOrientedButton)) {
            fieldOriented = !fieldOriented;
        }

        boolean slowModeButton = sharedInputValues.getBoolean(this.slowModeButton);
        boolean cornerModeButton = sharedInputValues.getBoolean(this.cornerModeButton);
        double xAxis = rangeStick(sharedInputValues.getNumeric(this.xAxis));
        double yAxis = rangeStick(sharedInputValues.getNumeric(this.yAxis));
        double rotateAxis = rangeStick(sharedInputValues.getNumeric(this.rotateAxis));

        if (slowModeButton) {
            currentMaxModuleVelocity = slowModeMaxVelocity;
        }
        else if (cornerModeButton) {
            currentMaxModuleVelocity = cornerModeMaxVelocity;
        } else {
            currentMaxModuleVelocity = maxModuleVelocity;
        }

        sharedInputValues.setNumeric("ipn_drivetrain_max_velocity", currentMaxModuleVelocity);

        // This is the orientation of the front of the robot based on the unit circle. It does not have to be 0.
        double robotOrientation = 0;

        // When using field orientation, forward is always towards the opposite end of the field even if the robot is facing a different direction.
        // To do this, the angle of the robot read from the navx is subtracted from the direction chosen by the driver.
        // For example, if the robot is rotated 15 degrees and the driver chooses straight forward, the actual angle is -15 degrees.
        if (fieldOriented) {
            robotOrientation += sharedInputValues.getVector("ipv_navx").get("angle");
        }

        // Swapping X and Y translates coordinate systems from the controller to the robot.
        // The controller use the Y axis for forward/backwards and the X axis for right/left
        // The robot forward/backwards is along the X axis and left/right is along the Y axis
        Vector translation = new Vector(new Point(yAxis, xAxis)).rotate(robotOrientation);

        if (targetLimelight){
            setModulePowers(translation, "limelight", 0);
        } else {
            setModulePowers(translation, rotateAxis);
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

    private double rangeStick(double value) {
        double valueSign = Math.signum(value);
        value = Math.abs(value);

        double min = 0.15;
        double max = 1.0;

        double minOutput = 0.0;
        double maxOutput = 1.0;

        double linearOutput = (((value - min) / (max - min)) * (maxOutput - minOutput) + minOutput) * valueSign;

        return linearOutput;
    }
}