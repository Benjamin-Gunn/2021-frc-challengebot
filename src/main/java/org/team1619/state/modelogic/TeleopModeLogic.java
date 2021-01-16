package org.team1619.state.modelogic;

import org.uacr.models.state.State;
import org.uacr.robot.AbstractModeLogic;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

/**
 * Handles the isReady and isDone logic for teleop mode on competition bot
 */

public class TeleopModeLogic extends AbstractModeLogic {

	private static final Logger sLogger = LogManager.getLogger(TeleopModeLogic.class);
	private int mode;

	public TeleopModeLogic(InputValues inputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, robotConfiguration);
	}

	@Override
	public void initialize() {
		sLogger.info("***** TELEOP *****");
		mode = 0;
	}

	@Override
	public void update() {
		if (fSharedInputValues.getBooleanFallingEdge("ipb_driver_back")){
			mode = (mode < 2)?(mode + 1): 0;
		}
	}

	@Override
	public void dispose() {

	}

	@Override
	public boolean isReady(String name) {
		switch (name) {
//			case "st_example_zero":
//				return !fSharedInputValues.getBoolean("ipb_example_has_been_zeroed");
			case "st_example":
				return fSharedInputValues.getBooleanRisingEdge("ipb_operator_right_trigger");
			case "st_drivetrain_zero":
				return !fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed");
			case "st_drivetrain_swerve_vector":
				return mode == 0;
			case "st_drivetrain_swerve_math":
				return mode == 1;
			case "st_drivetrain_swerve_matthew":
				return mode == 2;
			default:
				return false;
		}
	}

	@Override
	public boolean isDone(String name, State state) {
		switch (name) {
			default:
				return state.isDone();
		}
	}
}
