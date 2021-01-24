package org.team1619.modelfactory;

import org.team1619.behavior.*;
import org.uacr.models.behavior.Behavior;
import org.uacr.models.exceptions.ConfigurationException;
import org.uacr.robot.AbstractModelFactory;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.ObjectsDirectory;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

public class ModelFactory_Behaviors extends AbstractModelFactory {

	private static final Logger sLogger = LogManager.getLogger(ModelFactory_Behaviors.class);

	private final InputValues fSharedInputValues;
	private final OutputValues fSharedOutputValues;
	private final RobotConfiguration fRobotConfiguration;

	public ModelFactory_Behaviors(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration, ObjectsDirectory objectsDirectory) {
		super(inputValues, outputValues, robotConfiguration, objectsDirectory);
		fSharedInputValues = inputValues;
		fSharedOutputValues = outputValues;
		fRobotConfiguration = robotConfiguration;
	}

	public Behavior createBehavior(String name, Config config) {
		sLogger.trace("Creating behavior '{}' of type '{}' with config '{}'", name, config.getType(), config.getData());

		switch (name) {

			case "bh_drivetrain_zero":
				return new Behavior_Drivetrain_Zero(fSharedInputValues, fSharedOutputValues, config, fRobotConfiguration);
			case "bh_drivetrain_swerve_math":
				return new Behavior_Drivetrain_Swerve_Math(fSharedInputValues, fSharedOutputValues, config, fRobotConfiguration);
			case "bh_drivetrain_swerve_matthew":
				return new Behavior_Drivetrain_Swerve_Matthew(fSharedInputValues, fSharedOutputValues, config, fRobotConfiguration);
			case "bh_drivetrain_swerve_vector":
				return new Behavior_Drivetrain_Swerve_Vector(fSharedInputValues, fSharedOutputValues, config, fRobotConfiguration);
			case "bh_drivetrain_swerve_pure_pursuit":
				return new Behavior_Drivetrain_Swerve_Pure_Pursuit(fSharedInputValues, fSharedOutputValues, config, fRobotConfiguration);
			case "bh_drivetrain_swerve_align":
				return new Behavior_Drivetrain_Swerve_Align(fSharedInputValues, fSharedOutputValues, config, fRobotConfiguration);


			// State not found
			default:
				throw new ConfigurationException("Behavior " + name + " does not exist.");
		}
	}

}