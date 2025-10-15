# Reefscape OffSeason Codebase
[![CI](https://github.com/SMNWTeam1982/reefscape-offseason/actions/workflows/main.yml/badge.svg)](https://github.com/SMNWTeam1982/reefscape-offseason/actions/workflows/main.yml)
<br>
Based on the Java-Swerve-Template, the Reefscape OffSeason Codebase is a complete Java re-implementation of the 2025 Season Python Codebase, with robot subsystems re-written in the Command-Based paradigm.

!! Please do not tamper with the implementations provided by the Java-Swerve-Template in this repository, they are pulled from the Template !!

## Subsystems
 - [Swerve Drive Subsystem](https://github.com/SMNWTeam1982/java-swerve-template/blob/main/src/main/java/frc/robot/subsystems/swerve/DriveSubsystem.java) - Provided by Java-Swerve-Template
 - [Vision Subsystem(s)](https://github.com/SMNWTeam1982/java-swerve-template/blob/main/src/main/java/frc/robot/subsystems/vision/VisionSubsystem.java) - Provided by Java-Swerve-Template
 - [Elevator Subsystem](https://github.com/SMNWTeam1982/reefscape-offseason/blob/main/src/main/java/frc/robot/subsystems/Elevator/ElevatorSubsystem.java)
 - [Climber Subsystem](https://github.com/SMNWTeam1982/reefscape-offseason/blob/main/src/main/java/frc/robot/subsystems/climber/ClimberSubsystem.java)
 - [Coral Intake/Wrist Subsystem](https://github.com/SMNWTeam1982/reefscape-offseason/blob/main/src/main/java/frc/robot/subsystems/Wrist/IntakeSubsystem.java)
 - [Algae Intake Subsystem](https://github.com/SMNWTeam1982/reefscape-offseason/blob/main/src/main/java/frc/robot/subsystems/Algae/AlgaeSubsystem.java)

## Components
 - [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit/) - Advanced logging framework for replay and testing
 - [PathPlanner](https://github.com/mjansen4857/pathplanner) - Comprehensive autonomous design framework
 - [PhotonLib](https://photonvision.org/) - AprilTag recognition and vision system
 - [QuestNav](https://questnav.gg) - Oculus Quest 3s precision Odometry and position data
 - [REVLib](https://docs.revrobotics.com/revlib) & CTRE [Phoenix](https://v6.docs.ctr-electronics.com/en/stable/#) - Low level hardware control libraries
