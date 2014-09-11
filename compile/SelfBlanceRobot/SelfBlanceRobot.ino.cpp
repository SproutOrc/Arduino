#include <Arduino.h>
public class Button extends PApplet{

    
public Button (int x, int y, int width, int hight);
public Button();
public void setProperty(int x, int y, int width, int hight);
public void showButton();
public boolean isPressed(int x, int y);
public ConnectProtocol ();
public static void name();
void draw();

public Button (int x, int y, int width, int hight) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.hight = hight;
        this.pressed = false;  
    }

    public Button() {
        this.pressed = false;
    }

    public void setProperty(int x, int y, int width, int hight) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.hight = hight;
        this.pressed = false;
    }

    public void showButton() {
        ellipse(this.x, this.y, this.width, this.hight);
    }

    public boolean isPressed(int x, int y) {

        return true;
    }

    private boolean pressed;

    private int x;
    private int y;
    private int width;
    private int hight;

}public static class ConnectProtocol extends PApplet {

    public ConnectProtocol () {
        
    }

    public static void name() {
        
    }

}void setup() {
    
}

void draw() {
    
}