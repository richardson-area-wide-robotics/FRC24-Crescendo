package frc.lib.util;

public class JoystickUtil {
  public static double squareAxis(double input) {
    return (input * input) * Math.signum(input);
  }
}