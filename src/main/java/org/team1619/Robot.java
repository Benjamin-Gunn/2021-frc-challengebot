package org.team1619;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import org.team1619.modelfactory.RobotModelFactory;
import org.team1619.robot.FrcHardwareRobot;
import org.team1619.services.logging.LoggingService;
import org.team1619.state.RobotModule;
import org.team1619.state.StateControls;
import org.uacr.robot.AbstractModelFactory;
import org.uacr.robot.AbstractStateControls;
import org.uacr.robot.RobotCore;
import org.uacr.services.input.InputService;
import org.uacr.services.output.OutputService;
import org.uacr.services.states.StatesService;
import org.uacr.services.webdashboard.WebDashboardService;
import org.uacr.shared.abstractions.FMS;
import org.uacr.shared.concretions.SharedRobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.YamlConfigParser;
import org.uacr.utilities.injection.Injector;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import org.uacr.utilities.services.ScheduledMultiService;
import org.uacr.utilities.services.Scheduler;
import org.uacr.utilities.services.managers.AsyncServiceManager;
import org.uacr.utilities.services.managers.ServiceManager;

public class Robot extends TimedRobot {

	private final RobotCore fRobot;

	public Robot() {
		fRobot = new FrcHardwareRobot() {
			@Override
			protected AbstractStateControls createStateControls() {
				return new StateControls(fInputValues, fRobotConfiguration);
			}

			@Override
			protected AbstractModelFactory createModelFactory() {
				return new RobotModelFactory(fHardwareFactory, fInputValues, fOutputValues, fRobotConfiguration, fObjectsDirectory);
			}
		};
	}

	public static void main(String... args) {
		RobotBase.startRobot(Robot::new);
	}

	@Override
	public void robotInit() {
		fRobot.start();
	}

	@Override
	public void teleopInit() {
		fRobot.getFms().setMode(FMS.Mode.TELEOP);
	}

	@Override
	public void autonomousInit() {
		fRobot.getFms().setMode(FMS.Mode.AUTONOMOUS);
	}

	@Override
	public void disabledInit() {
		fRobot.getFms().setMode(FMS.Mode.DISABLED);
	}

	@Override
	public void testInit() {
		fRobot.getFms().setMode(FMS.Mode.TEST);
	}
}