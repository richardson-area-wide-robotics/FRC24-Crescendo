package frc.lib.controller;

public class PIDGains{
  public PIDGains(double p, double i, double d) {
    P = p;
    I = i;
    D = d;
  }

  public PIDGains() {}

  public double P = 0.0;
  public double I = 0.0;
  public double D = 0.0;
}