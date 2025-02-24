package org.tahomarobotics.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.tahomarobotics.robot.auto.Autonomous;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.climber.Climber;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.grabber.Grabber;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.shims.FauxWatchdog;
import org.tahomarobotics.robot.vision.Vision;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.commands.WindmillTrajectories;
import org.tinylog.Logger;

import java.util.List;

@Logged
public class Robot extends TimedRobot {
    // Subsystems

    @Logged(name = "Chassis")
    private final Chassis chassis = Chassis.getInstance();
    @Logged(name = "Autonomous")
    private final Autonomous autonomous = Autonomous.getInstance();
    @Logged(name = "Vision")
    private final Vision vision = Vision.getInstance();
    @Logged(name = "Windmill")
    private final Windmill windmill = Windmill.getInstance();
    @Logged(name = "Collector")
    private final Collector collector = Collector.getInstance();
    @Logged(name = "Indexer")
    private final Indexer indexer = Indexer.getInstance();
    @Logged(name = "Grabber")
    private final Grabber grabber = Grabber.getInstance();
    @Logged(name = "Climber")
    private final Climber climber = Climber.getInstance();
    @Logged(name = "OI")
    private final OI oi = OI.getInstance();

    @NotLogged
    private final List<SubsystemIF> subsystems = List.of(
        chassis.initialize(),
        autonomous.initialize(),
        vision.initialize(),
        windmill.initialize(),
        indexer.initialize(),
        collector.initialize(),
        climber.initialize(),
        grabber.initialize(),
        oi.initialize()
    );

    // Robot

    public Robot() {
        Epilogue.configure(configuration -> {
            configuration.backend = configuration.backend.lazy();
            configuration.minimumImportance = Logged.Importance.DEBUG; // TODO
        });
        Epilogue.bind(this);

        WindmillTrajectories.initialize();

//        disableWatchdog(`this, IterativeRobotBase.class);
//        disableWatchdog(CommandScheduler.getInstance(), CommandScheduler.class);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    // Disabled

    @Override
    public void disabledInit() {
        subsystems.forEach(SubsystemIF::onDisabledInit);
    }

    @Override
    public void disabledPeriodic() {}

    // Autonomous

    @Override
    public void autonomousInit() {
        subsystems.forEach(SubsystemIF::onAutonomousInit);
        autonomous.getSelectedAuto().schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    // Teleop

    @Override
    public void teleopInit() {
        subsystems.forEach(SubsystemIF::onTeleopInit);
    }

    @Override
    public void teleopPeriodic() {}

    // Test

    @Override
    public void testInit() {
        oi.initializeSysId();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {
        oi.cleanUpSysId();
    }

    // Simulation

    @Override
    public void simulationInit() {
        subsystems.forEach(SubsystemIF::onSimulationInit);
    }

    @Override
    public void simulationPeriodic() {}

    // Util

    <T> void disableWatchdog(T inst, Class<?> clazz) {
        try {
            var field = clazz.getDeclaredField("m_watchdog");
            field.setAccessible(true);
            field.set(inst, new FauxWatchdog());
            Logger.info("Disabled {}'s watchdog!", inst.getClass());
        } catch (NoSuchFieldException | IllegalAccessException ignored) {}
    }
}
