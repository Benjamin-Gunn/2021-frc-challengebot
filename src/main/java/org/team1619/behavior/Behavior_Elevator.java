package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.Set;

/**
 * Runs elevator at specified speed
 */

public class Behavior_Elevator implements Behavior {

	private static final Logger sLogger = LogManager.getLogger(Behavior_Elevator.class);
	private static final Set<String> sSubsystems = Set.of("ss_elevator");

	private final InputValues fSharedInputValues;
	private final OutputValues fSharedOutputValues;

	private double speed;

	public Behavior_Elevator(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
		fSharedInputValues = inputValues;
		fSharedOutputValues = outputValues;

		speed = 0.0;
	}

	@Override
	public void initialize(String stateName, Config config) {
		sLogger.debug("Entering state {}", stateName);

		speed = config.getDouble("speed", 0.0);
	}

	@Override
	public void update() {
		fSharedOutputValues.setNumeric("opn_elevator","percent", speed);
	}

	@Override
	public void dispose() {
		fSharedOutputValues.setNumeric("opn_elevator", "percent", 0);
	}

	@Override
	public boolean isDone() {
		return true;
	}

	@Override
	public Set<String> getSubsystems() {
		return sSubsystems;
	}
}