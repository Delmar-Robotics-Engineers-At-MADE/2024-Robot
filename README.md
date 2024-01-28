# 8077 DRE@M 2024 Robot

See [the online changelog](https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/CHANGELOG.md) for information about updates to the template that may have been released since you created your project.

## Description

This is intended for hardware based on Unqualified Quokkas.

Controls document can be found here: https://docs.google.com/document/d/1kfDaiIaLO5A7UbvRqy12fkUvl5K5Y3AVDwxTCp0MVLY/edit?usp=sharing
This robot uses all Rev motors, motor controllers and control system components and as such does not contain Phoenix v5 or Phoenix 6.

m_driverController is designed as a Logitech Extreme 3D PRO.
m_operatorController is designed as a Logitech F310 in X setting.

Robot CAD: [https://cad.onshape.com/documents/aad7fd6f877dc6ed87ba34ed/w/db49167d2d3adc31ac0e2ba5/e/a6a6c869a7619e9ebe9bd458?renderMode=0&uiState=65b65e475b440d6e0ce2c375](https://cad.onshape.com/documents/aad7fd6f877dc6ed87ba34ed/w/db49167d2d3adc31ac0e2ba5/e/a6a6c869a7619e9ebe9bd458?renderMode=0&uiState=65b65e7f5b440d6e0ce2c8df)

## Prerequisites

* SPARK MAX Firmware v1.6.2 - Adds features that are required for swerve
* REVLib v2023.1.2 - Includes APIs for the new firmware features

## Configuration

It is possible that this project will not work for your robot right out of the box. Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined for your own robot!

These values can be adjusted in the `Constants.java` file.
