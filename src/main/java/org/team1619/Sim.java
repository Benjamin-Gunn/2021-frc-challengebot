package org.team1619;

import org.team1619.modelfactory.SimModelFactory;
import org.team1619.robot.FrcSimRobot;
import org.team1619.services.logging.LoggingService;
import org.team1619.state.SimModule;
import org.team1619.state.StateControls;
import org.uacr.robot.AbstractModelFactory;
import org.uacr.robot.AbstractStateControls;
import org.uacr.services.input.InputService;
import org.uacr.services.output.OutputService;
import org.uacr.services.states.StatesService;
import org.uacr.services.webdashboard.WebDashboardService;
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

public class Sim {

	public static void main(String[] args) {
		new FrcSimRobot() {
			@Override
			protected AbstractStateControls createStateControls() {
				return new StateControls(fInputValues, fRobotConfiguration);
			}

			@Override
			protected AbstractModelFactory createModelFactory() {
				return new SimModelFactory(fHardwareFactory, fEventBus, fInputValues, fOutputValues, fRobotConfiguration, fObjectsDirectory);
			}
		}.start();
	}
}
