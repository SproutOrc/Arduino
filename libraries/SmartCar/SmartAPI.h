class SmartAPI
{
public:
    SmartAPI();
    ~SmartAPI();
    void left ();
    void right ();
    void straight ();
    void back ();

private:
    byte leftWheelSpeed;
    byte rightWheelSpeed;
};