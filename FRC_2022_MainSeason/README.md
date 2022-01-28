# **FRC 2021 Main Season Robot**

# Team Roles

(todo)

 - **Drivetrain**
 - **Teleop**
 - **Turret**
 - **Hooker**
 - **Vision & Autonomous**

<br/>

# Conventions

All classes should have a comment above their definition explaining what the class does. (Explain what the class does)

All code that is not [self documented]("https://en.wikipedia.org/wiki/Self-documenting_code#:~:text=Self%2Ddocumenting%20code%20is%20ostensibly,symbol's%20meaning%2C%20such%20as%20article.&text=The%20code%20must%20also%20have,easily%20understand%20the%20algorithm%20used.") should be commented.

All math that uses formulas should have the process of how the result is derived.

All code should be properly indented, spaced, and formatted.

<br/>

# Project Layout

All code is within *"./src/java/"*

## Important Files

All global constants (Physical ports on the robot and tuned coefficients) are in *"./Constants.java"*

All subsystem references are defined in *"./RobotContainer.java"*

## Important Directories

All code that directly interfaces with hardware subsystems is in *"./subsystems/"*

All code for commands that are called manually are in *"./commands/"*

All code for commands that are called autonomously are in *"./autonomous/"*