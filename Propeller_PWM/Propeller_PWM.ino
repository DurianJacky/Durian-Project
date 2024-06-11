
#define pwmPin1 9 // 电机（左边）的PWM输出引脚
#define pwmPin2 10 // 电机（右边）的PWM输出引脚

int direction1 = 0; // 电机1的方向
int velocity1 = 0; // 电机1的速度百分比
int direction2 = 0; // 电机2的方向
int velocity2 = 0; // 电机2的速度百分比

void setup() {
    // 初始化串口通讯
    Serial.begin(115200);
  
    // 设置引脚模式
    pinMode(pwmPin1, OUTPUT);
    pinMode(pwmPin2, OUTPUT);
    analogWrite(pwmPin1, 128);
    analogWrite(pwmPin2, 128);
}

void loop() {
    int pwmValue1 = 0;
    int pwmValue2 = 0;
    if (Serial.available() >= 4) { // 检查是否接收到至少四个字节的数据
        direction1 = Serial.read(); // 读取电机1的方向字节
        velocity1 = Serial.read(); // 读取电机1的速度字节
        direction2 = Serial.read(); // 读取电机2的方向字节
        velocity2 = Serial.read(); // 读取电机2的速度字节
        

    
        if (direction1 == 1) {
            // 正转
            pwmValue1 = 128 + velocity1;
        } else {
            // 反转
            pwmValue1 = 128 - velocity1;
        }

        
        if (direction2 == 1) {
            // 正转
            pwmValue2 = 128 + velocity2;
        } else {
            // 反转
            pwmValue2 = 128 - velocity2;
        }

        // 输出PWM值
        analogWrite(pwmPin1, pwmValue1);
        analogWrite(pwmPin2, pwmValue2);

        Serial.print("Motor 1 : ");
        Serial.print(pwmValue1);
        Serial.print(" ｜Motor 2 : ");
        Serial.println(pwmValue2);
    }

        
}
