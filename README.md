CMonster2016
============

Java robot code for FRC team 2084, Robots By The C

#### This code contains a few features that might be useful for other teams:

* Commands that accepts parameters from the SmartDashboard ([ParameterCommand.java](src/main/java/org/usfirst/frc/team2084/CMonster2016/commands/ParameterCommand.java))
* A flexible NetworkTables based [parameter system](src/main/java/org/usfirst/frc/team2084/CMonster2016/parameters) that works with our [web interface](../../../WebDashboard2016)
* Aiming code that takes commands from our [vision algorithm](../../../VisionProcessor2016) running on a NVIDIA Jetson TK1
* **Trajectory generation and following using [Pathfinder](../../../../JacisNonsense/Pathfinder)**
* Velocity PID feedback for drivetrain control
* Example of using Java 8 features (lambdas, method references) in robot code
* Code for sending arrays with more than 255 items over NetworkTables ([NetworkTablesLargeArrays.java](src/main/java/org/usfirst/frc/team2084/CMonster2016/util/NetworkTablesLargeArrays.java))
* Use of Gradle as a build system
* Basic unit testing for a few features

#### Our other 2016 repositories:
* [VisionProcessor2016](../../../VisionProcessor2016) - Implementation of our computer vision algorithms
* [VisionServer2016](../../../VisionServer2016) - Launching code for our vision system on our NVIDIA Jetson TK1
* [VisionTest2016](../../../VisionTest2016) - Application for testing and calibrating our vision code
* [SmartDashboardExtensions2016](../../../SmartDashboardExtensions2016) - Our custom SmartDashboard extensions
* [WebDashboard2016](../../../WebDashboard2016) - Our web dashboard that makes tuning and debugging easy (uses [pynetworktables2js](../../../../robotpy/pynetworktables2js))