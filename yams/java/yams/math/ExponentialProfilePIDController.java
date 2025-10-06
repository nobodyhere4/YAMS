package yams.math;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

/**
 * Exponential profile PID controller. Similar to {@link PIDController} or
 * {@link edu.wpi.first.math.controller.ProfiledPIDController}, but uses an {@link ExponentialProfile}
 */
public class ExponentialProfilePIDController
{

  /**
   * The wrapped PID controller.
   */
  private       PIDController                      controller;
  /**
   * The wrapped profile.
   */
  private       ExponentialProfile                 profile;
  /**
   * The current state from {@link ExponentialProfile}
   */
  private       ExponentialProfile.State           currentState = new State();
  /**
   * The next state from {@link ExponentialProfile}
   */
  private       Optional<ExponentialProfile.State> nextState    = Optional.empty();
  /**
   * Iteration timer.
   */
  private final Timer                              timer        = new Timer();
  /**
   * Loop time.
   */
  private       Time                               loopTime     = Milliseconds.of(20);

  /**
   * Constructor.
   *
   * @param controller The wrapped PID controller.
   * @param profile    The wrapped profile.
   */
  public ExponentialProfilePIDController(PIDController controller, ExponentialProfile profile)
  {
    this.controller = controller;
    this.profile = profile;
  }

  /**
   * Constructor.
   *
   * @param kP          kP value for the {@link PIDController}
   * @param kI          kI value for the {@link PIDController}
   * @param kD          kD value for the {@link PIDController}
   * @param constraints {@link Constraints} for the {@link ExponentialProfile}
   */
  public ExponentialProfilePIDController(double kP, double kI, double kD, Constraints constraints)
  {
    this(new PIDController(kP, kI, kD), new ExponentialProfile(constraints));
  }

  /**
   * Reset the controller, set the next setpoint to empty.
   */
  public void reset()
  {
    controller.reset();
    currentState = new State();
    nextState = Optional.empty();
  }

  /**
   * Get the setpoint.
   *
   * @return setpoint.
   */
  public double getSetpoint()
  {
    return controller.getSetpoint();
  }

  /**
   * Get the current state of the {@link ExponentialProfile}.
   *
   * @return {@link ExponentialProfile.State} given by the {@link ExponentialProfile}.
   */
  public State getCurrentState()
  {
    return currentState;
  }

  /**
   * Get the next state of the {@link ExponentialProfile}.
   *
   * @return {@link ExponentialProfile.State} given by the {@link ExponentialProfile}.
   */
  public Optional<State> getNextState()
  {
    return nextState;
  }

  /**
   * Calculate the feedback, assuming previous state velocity.
   *
   * @param measurementPosition Measurement position to set as the current state..
   * @param setpointVelocity    Setpoint velocity.
   * @param setpointPosition    Setpoint position.
   * @return
   */
  public double calculate(double measurementPosition, double setpointVelocity, double setpointPosition)
  {
    if (timer.isRunning())
    {
      loopTime = Seconds.of(timer.get());
    }
    timer.reset();
    timer.start();
    var feedback = controller.calculate(measurementPosition, currentState.position);
    nextState.ifPresent(state -> currentState = state);
    nextState = Optional.of(profile.calculate(loopTime.in(Seconds),
                                              currentState,
                                              new State(setpointPosition, setpointVelocity)));
    return feedback;
  }

  /**
   * Calculate the feedback, assuming no setpoint velocity.
   *
   * @param measurementPosition Measurement position to set as the current state.
   * @param setpointPosition    Setpoint position.
   * @return
   */
  public double calculate(double measurementPosition, double setpointPosition)
  {
    return calculate(measurementPosition, 0, setpointPosition);
  }

}
