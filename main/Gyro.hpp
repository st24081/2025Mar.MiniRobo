
#pragma once

class Gyro
{
public:
    Gyro(Udon::BNO055 Driver_)
        : gyro(std::move(Driver_))
        , angle(0)
        , lastAngle(0)
        , offset(0)
    {
    }

    void begin() { gyro.begin(); }

    void update()
    {
        gyro.update();

        double realAngle = gyro.getQuaternion().toYaw();

        angle = wrapAngle(realAngle, lastAngle);
        // ジャイロセンサは Pi ~ -Pi で返ってくる(絶対角度)なので、
        // Pi -> -Pi のときや -Pi -> Pi のときに注意しないと暴走する

        lastAngle = realAngle;
    }

    void clear()
    {
        gyro.clear();
        angle  = 0;
        offset = 0;
    }

    double getAngle() { return angle; }

private:
    Udon::BNO055 gyro;
    double       angle;
    double       lastAngle;
    double       offset;

    double wrapAngle(double current, double last)
    {
        if (std::fabs(current - last) > Udon::Pi)    // 急にPi以上回転したとき->切れ目のところを通過したと判断
        {
            if (current > 0)
            {
                offset -= Udon::TwoPi;
            }
            else if (current < 0)
            {
                offset += Udon::TwoPi;
            }
        }
        return current + offset;
    }
};
 
