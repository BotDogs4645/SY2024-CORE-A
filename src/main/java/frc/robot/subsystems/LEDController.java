// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDController extends SubsystemBase {
  public static enum LEDColor {
    PINK(0.57),
    WHITE(0.93),
    RED(0.61),
    BLUE(0.83),
    GREEN(0.75),
    ORANGE(0.65);

    private double PWMValue;
    private LEDColor(double PWMValue) {
      this.PWMValue = PWMValue;
    }

    public double getPWMValue() {
      return PWMValue;
    }

  }

  private Spark controller;
  public LEDColor currentColor;

  public LEDController() {
    this.controller = new Spark(Constants.ledControllerID);
    setLEDColor(LEDColor.WHITE);
  }

  public void setLEDColor(LEDColor color) {
    this.currentColor = color;
    controller.set(this.currentColor.getPWMValue());
  }

  public LEDColor getColor() {
    return currentColor;
  }

  @Override
  public void periodic() {}
}