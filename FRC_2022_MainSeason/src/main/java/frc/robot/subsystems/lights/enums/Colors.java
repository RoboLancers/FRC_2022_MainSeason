package frc.robot.subsystems.lights.enums;

public enum Colors {
    YELLOW  (218, 235, 33), 
    BLUE    (33, 121, 235), 
    GREEN   (23, 194, 26), 
    RED     (194, 23, 23);

    public final int red;
    public final int green;
    public final int blue;

    Colors(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }
}