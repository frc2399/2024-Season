package frc.utils;

public class SimEncoder {

  private double distance = 0.0;
  private double speed = 0.0;

  public SimEncoder(String name) {
  }
  
  /**
   * Get the speed of the encoder.
   *
   * @return The speed of the encoder in whatever units the user used when
   *     calling setSpeed.
   */
  public double getSpeed() {
    return speed;
  }

  /**
   * Get the distance of the encoder.
   *
   * @return The distance of the encoder in whatever units the user used when
   *     calling setDistance.
   */
  public double getDistance() {
    return distance;
  }

  /**
   * Set the speed of the encoder.
   *
   * @param speed Speed of the encoder in unit's of the users choice.
   */
  public void setSpeed(double speed) {
    this.speed = speed;
  }

  /**
   * Set the distance of the encoder.
   *
   * @param distance Distance of the encoder in unit's of the users choice.
   */
  public void setDistance(double distance) {
    this.distance = distance;
  }
}
