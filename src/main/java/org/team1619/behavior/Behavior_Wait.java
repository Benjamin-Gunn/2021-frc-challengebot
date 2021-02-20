package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.Timer;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.Set;

/**
 * Example behavior to copy for other behaviors
 */

public class Behavior_Wait implements Behavior {

	private static final Logger sLogger = LogManager.getLogger(Behavior_Wait.class);
	private static final Set<String> sSubsystems = Set.of("ss_drivetrain");


	public Behavior_Wait(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
	}

	@Override
	public void initialize(String stateName, Config config) {
		sLogger.debug("Entering state {}", stateName);
	}

	@Override
	public void update() {
	}

	@Override
	public void dispose() {

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