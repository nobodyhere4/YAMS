package yams.helpers;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

;
//https://github.com/robototes/2020_Template/blob/main/src/test/java/frc/team2412/robot/subsystems/ExampleSubsystemTest.java
/**
 * Extend this class when your test requires commands or command groups to be
 * exercised with the full WPI scheduler. Use
 * {@link SchedulerPumpHelper#runForDuration(int, int...)} to pump the
 * scheduler.
 */
public class TestWithScheduler {

	public static void schedulerStart() {
		CommandScheduler.getInstance().enable();
	}

	public static void schedulerClear() {
		CommandScheduler.getInstance().cancelAll();
	}

	public static void schedulerDestroy() {
		CommandScheduler.getInstance().disable();
	}
}