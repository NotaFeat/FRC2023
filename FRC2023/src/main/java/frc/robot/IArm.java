package frc.robot;

public interface IArm {

    public void init();

    public void stop();
    
    public void shoulderUp();

    public void shoulderDown();

    public void elbowUp();

    public void elbowDown();

    public void grabberClose();

    public void grabberOpen();

    public void autoArm(Runnable completionRoutine);

    public void periodic();

}
