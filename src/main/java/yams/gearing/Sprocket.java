package yams.gearing;

import yams.exceptions.NoStagesGivenException;

/**
 * Sprocket class to handle calculating the conversion factor of a sprocket in your mechanism.
 */
public class Sprocket
{

  /**
   * Stages in the Sprocket chain.
   */
  private final double[] reductionStages;
  /**
   * The input to output conversion factor.
   */
  private final double   sprocketReductionRatio;

  /**
   * Create the sprocket given the teeth of each sprocket in the chain.
   *
   * @param sprocketReductionStage Sprocket teeth.
   */
  public Sprocket(double[] sprocketReductionStage)
  {
    reductionStages = sprocketReductionStage;
    if (reductionStages.length == 0)
    {
      throw new NoStagesGivenException();
    }
    double sprocketRatio = reductionStages[0];
    for (int i = 1; i < reductionStages.length; i++)
    {
      sprocketRatio *= reductionStages[i];
    }
    sprocketReductionRatio = sprocketRatio;
  }

  /**
   * Get the conversion factor to transform the sprocket input into the sprocket output rotations.
   *
   * @return OUT/IN or OUT:IN
   */
  public double getInputToOutputConversionFactor()
  {
    return 1 / sprocketReductionRatio;
  }

  /**
   * Get the conversion factor to transform the sprocket output value into the sprocket input value.
   *
   * @return IN:OUT or IN/OUT
   */
  public double getOutputToInputConversionFactor()
  {
    return sprocketReductionRatio;
  }


}
