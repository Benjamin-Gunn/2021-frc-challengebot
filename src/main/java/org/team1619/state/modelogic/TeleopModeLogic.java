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
//		if (fSharedInputValues.getBooleanFallingEdge("ipb_driver_back")){
//			mode = (mode < 2)?(mode + 1): 0;
//		}
	}

	@Override
	public void dispose() {

	}

	@Override
	public boolean isReady(String name) {
		switch (name) {
			case "st_drivetrain_zero":
				return !fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed");
			case "st_drivetrain_swerve":
				return mode == 0;
			case "st_drivetrain_swerve_math":
				return mode == 1;
			case "st_drivetrain_swerve_align_test":
				return fSharedInputValues.getBoolean("ipb_driver_b");
			case "st_elevator_shoot":
				return fSharedInputValues.getBoolean("ipb_driver_x");
			case "st_elevator_eject":
				return fSharedInputValues.getBoolean("ipb_driver_y");
			case "st_elevator_stop":
				return fSharedInputValues.getBoolean("ipb_driver_a");
			default:
				return false;
		}
	}

	@Override
	public boolean isDone(String name, State state) {
		switch (name) {
			case "st_drivetrain_swerve_align_test":
				return !fSharedInputValues.getBoolean("ipb_driver_b");
			default:
				return state.isDone();
		}
	}
}
