package yams.gearing.gearbox;

import edu.wpi.first.math.system.plant.DCMotor;
import yams.exceptions.NoStagesGivenException;

/**
 * GearBox class to calculate input and output conversion factors and check if the current configuration is supported.
 */
public abstract class GearBox
{

  /**
   * Stages in the gear box
   */
  private final double[] stages;
  /**
   * Conversion factor of the gearbox from input to output.
   */
  private final double   conversionFactor;
  /**
   * Construct the {@link GearBox} with the stages given.
   *
   * @param stage List of stages in the gearbox.
   */
  public GearBox(double[] stage)
  {
    this.stages = stage;
    if (stages.length == 0)
    {
      throw new NoStagesGivenException();
    }
    double gearBox = stages[0];
    for (int i = 1; i < stages.length; i++)
    {
      gearBox *= stages[i];
    }
    conversionFactor = gearBox;
  }

  /**
   * Get the conversion factor to transform the gearbox input into the gear box output rotations.
   *
   * @return OUT/IN or OUT:IN
   */
  public double getInputToOutputConversionFactor()
  {
    return 1 / conversionFactor;
  }

  /**
   * Get the conversion factor to transform the gearbox output value into the gear box input value.
   *
   * @return IN:OUT or IN/OUT
   */
  public double getOutputToInputConversionFactor()
  {
    return conversionFactor;
  }

  /**
   * Check the gearbox limit against the current mechanism.
   *
   * @param outputConversionFactor Additional conversion factor to apply before checking the limit
   * @param weight                 Weight of the mechanism.
   * @param motorModel             Primary motor for the mechanism.
   * @return Whether the configuration exceeds the limits of the MAXPlanetaryGearBox.
   */
  public abstract boolean checkGearBoxLimit(double outputConversionFactor, double weight, DCMotor motorModel);

  /**
   * Gear box types.
   */
  public enum Type
  {
    /**
     * MAXPlanetary gearbox
     */
    MAX_PLANETARY,
    /**
     * VersaPlanetary gearbox.
     */
    VERSA_PLANETARY
  }
}
