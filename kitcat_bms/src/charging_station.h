#ifndef CHARGING_STATION
#define CHARGING_STATION

class ChargingStation {
    public:
        ChargingStation();
        void setEsc(bool state);
        bool isEscEnabled();
        bool areBatteriesCharging();

    private:
        bool ESC;
        bool batteriesCharging;
};

#endif // CHARGING_STATION