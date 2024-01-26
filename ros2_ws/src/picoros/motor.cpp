include <Arduino.h>

class Motor{
    public:
        int enable;
        int I1;
        int I2;
        Motor(int x, int y,int z){// sets pins to correct outputs
            enable = x;
            I1 = y;
            I2 = z;
            pinMode(enable, OUTPUT);
            pinMode(I1, OUTPUT);
            pinMode(I2, OUTPUT);
        }
        void run(float speed) {//runs motor .run(speed) if speed is neg it will go backwards
            if(speed< 0){
                digitalWrite(-speed, enable);
                digitalWrite(I1, HIGH);
                digitalWrite(I2, LOW);
            }
            else{
                digitalWrite(speed, enable);
                digitalWrite(I1, LOW);
                digitalWrite(I2, HIGH);}
        }
        
}
class Prox_sensor{
    public:
        int pin;
        float distance;
        Prox_sensor(int x){
            pin = x;
            pinMode(pin,INPUT);
        }
        float read(){
            distance = analogRead(pin);
            return distance;

        }

}

class Encoders{
    public:
        int a;
        int b;
        Encoders(int x , int b){
            int a = x;
            int b = b;
            pinMode(a,INPUT);
            pinMode(b,INPUT);
        }
        // add interupts here
}


int main(){
    Motor Motor_l(6,9,10);
    Motor Motor_r(7,12,11);

    Motor_l.run()
}
