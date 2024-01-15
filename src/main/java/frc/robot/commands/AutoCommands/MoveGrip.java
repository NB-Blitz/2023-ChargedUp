// package frc.robot.commands.AutoCommands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.GripSubsystem;

// public class MoveGrip extends Command {
//     private int dir;
//     private double time;
//     private Timer deltaTime;
//     private GripSubsystem gripSubsystem;
    
//     public MoveGrip(int dir, double time, GripSubsystem gripSubsystem) {
//         this.dir = dir;
//         this.time = time;
//         this.deltaTime = new Timer();
//         this.gripSubsystem = gripSubsystem;
//     }

//     @Override
//     public void initialize() {
//         deltaTime.restart();
//     }

//     @Override
//     public void execute() {
//         gripSubsystem.move(dir);
//     }

//     @Override
//     public boolean isFinished() {
//         return deltaTime.get() >= time;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         gripSubsystem.move(0);
//         deltaTime.stop();
//     }
// }
