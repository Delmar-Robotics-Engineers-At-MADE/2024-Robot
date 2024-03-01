# 8077 DRE@M 2024 Robot

See [the online changelog](https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/CHANGELOG.md) for information about updates to the template that may have been released since you created your project.

## Description

This is intended for hardware based on Unqualified Quokkas.

Controls document can be found here: https://docs.google.com/document/d/1kfDaiIaLO5A7UbvRqy12fkUvl5K5Y3AVDwxTCp0MVLY/edit?usp=sharing
This robot uses all Rev motors, motor controllers and control system components and as such does not contain Phoenix v5 or Phoenix 6.

m_driverController is designed as a Logitech Extreme 3D PRO.
m_operatorController is designed as a Logitech F310 in X setting.

## Robot CAD 
  [Onshape](https://cad.onshape.com/documents/aad7fd6f877dc6ed87ba34ed/w/db49167d2d3adc31ac0e2ba5/e/a6a6c869a7619e9ebe9bd458?renderMode=0&uiState=65b65e7f5b440d6e0ce2c8df)

## Calculations
  [0 m from SUBWOOFER](https://www.desmos.com/3d/555efabc5c)
  
  [1.304 m from SUBWOOFER](https://www.desmos.com/3d/cbf728ffa3)

  [Intake Feedforward](https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A20%2C%22u%22%3A%22A%22%7D&efficiency=70100&flywheelMomentOfInertia=%7B%22s%22%3A3%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A1.5%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%20550%22%7D&motorRatio=%7B%22magnitude%22%3A5%2C%22ratioType%22%3A%22Reduction%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A0.8%2C%22u%22%3A%22lbs%22%7D&shooterMomentOfInertia=%7B%22s%22%3A1.4%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A2300%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A0.7%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=0&useCustomShooterMoi=0)

## Prerequisites

* SPARK MAX Firmware v1.6.2 - Adds features that are required for swerve
* REVLib v2023.1.2 - Includes APIs for the new firmware features

## Configuration

It is possible that this project will not work for your robot right out of the box. Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined for your own robot!

These values can be adjusted in the `Constants.java` file.
