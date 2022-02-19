package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.NotifierCommand;

public class StreamOutput extends NotifierCommand {
    public StreamOutput(Runnable updateSmartDashboard){
        super(updateSmartDashboard, 0.5);
    }

    @Override
    public void end(boolean interrupted){
        this.m_notifier.close();
    }
}
