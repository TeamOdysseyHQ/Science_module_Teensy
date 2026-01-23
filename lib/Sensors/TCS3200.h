#ifndef TCS3200_H
#define TCS3200_H


class TCS3200_Sensor {
public:
    TCS3200_Sensor(int s0, int s1, int s2, int s3, int outPin);
    void begin();
    bool isColorViolet();
private:
    unsigned long readColor(bool s2, bool s3);
    unsigned long redFreq, greenFreq, blueFreq;
    int s0, s1, s2, s3, outPin;
};  
#endif