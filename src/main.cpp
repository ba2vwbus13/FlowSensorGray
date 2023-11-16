/*
*******************************************************************************
* Copyright (c) 2021 by M5Stack
*                  Equipped with M5Core sample source code
*                          配套  M5Core 示例源代码
* Visit the website for more information：https://docs.m5stack.com/en/core/gray
* 获取更多资料请访问：https://docs.m5stack.com/zh_CN/core/gray
*
* describe：bmm150--Magnetometer 三轴磁力计
* date：2021/7/21
*******************************************************************************
*/

#include <Arduino.h>
#include "Preferences.h"
#include "M5Stack.h"
#include "math.h"
#include "M5_BMM150.h"
#include "M5_BMM150_DEFS.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <string.h>

//#incluce "LoRa.h"
//#define SDA_pin 32
//#define SDL_pin 33

int flow_sensor_number = 3;

WebServer server(80);  // WebServerオブジェクトを作る

//曲げセンサー定義
const int CH_PIN = 36; // ピン番号
const float MAX_VOLTAGE = 5; // 5Vを電源とした場合
const float ANALOG_MAX = 4095; // ESP32の場合

void drawLineByAngle(int16_t x, int16_t y, int16_t start, int16_t length, int16_t angle, int thick, uint16_t color);

Preferences prefs;

struct bmm150_dev dev;
bmm150_mag_data mag_offset; // Compensation magnetometer float data storage 储存补偿磁强计浮子数据
bmm150_mag_data mag_max;
bmm150_mag_data mag_min;
TFT_eSprite img = TFT_eSprite(&M5.Lcd);

int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len){
    if(M5.I2C.readBytes(dev_id, reg_addr, len, read_data)){ // Check whether the device ID, address, data exist.
        return BMM150_OK;                                   //判断器件的Id、地址、数据是否存在
    }else{
        return BMM150_E_DEV_NOT_FOUND;
    }
}

int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len){
    if(M5.I2C.writeBytes(dev_id, reg_addr, read_data, len)){    //Writes data of length len to the specified device address.
        return BMM150_OK;                                       //向指定器件地址写入长度为len的数据
    }else{
        return BMM150_E_DEV_NOT_FOUND;
    }
}

int8_t bmm150_initialization(){
    int8_t rslt = BMM150_OK;

    dev.dev_id = 0x10;  //Device address setting.  设备地址设置
    dev.intf = BMM150_I2C_INTF; //SPI or I2C interface setup.  SPI或I2C接口设置
    dev.read = i2c_read;    //Read the bus pointer.  读总线指针
    dev.write = i2c_write;  //Write the bus pointer.  写总线指针
    dev.delay_ms = delay;

    // Set the maximum range range
    //设置最大范围区间
    mag_max.x = -2000;
    mag_max.y = -2000;
    mag_max.z = -2000;

    // Set the minimum range
    //设置最小范围区间
    mag_min.x = 2000;
    mag_min.y = 2000;
    mag_min.z = 2000;

    rslt = bmm150_init(&dev);   //Memory chip ID.  存储芯片ID
    dev.settings.pwr_mode = BMM150_NORMAL_MODE;
    rslt |= bmm150_set_op_mode(&dev);   //Set the sensor power mode.  设置传感器电源工作模式
    dev.settings.preset_mode = BMM150_PRESETMODE_ENHANCED;
    rslt |= bmm150_set_presetmode(&dev);    //Set the preset mode of .  设置传感器的预置模式
    return rslt;
}

void bmm150_offset_save(){  //Store the data.  存储bmm150的数据
    prefs.begin("bmm150", false);
    prefs.putBytes("offset", (uint8_t *)&mag_offset, sizeof(bmm150_mag_data));
    prefs.end();
}

void bmm150_offset_load(){  //load the data.  加载bmm150的数据
    if(prefs.begin("bmm150", true)){
        prefs.getBytes("offset", (uint8_t *)&mag_offset, sizeof(bmm150_mag_data));
        prefs.end();
        Serial.println("bmm150 load offset finish....");
    }else{
        Serial.println("bmm150 load offset failed....");
    }
}

void drawLineByAngle(int16_t x, int16_t y, int16_t start, int16_t length, int16_t angle, int thick, uint16_t color) {
    float x0 = x + start * sin(radians(angle));
    float y0 = y - start * cos(radians(angle));
    float x1 = x + (start + length) * sin(radians(angle));
    float y1 = y - (start + length) * cos(radians(angle));
    for (int i = 0; i < thick; i++) {
        M5.Lcd.drawLine(x0, y0 - i, x1, y1 - i, color);
    }
}

void drawGrid() {
    for (int i = 0; i < 360; i += 5) {  // 5°ごとの目盛りを描く
        drawLineByAngle(160, 120, 100 - 10, 10, i, 1, WHITE);
    }
    for (int i = 0; i < 360; i += 30) {  // 30°ごとの目盛りを描く
        drawLineByAngle(160, 120, 100 - 15, 15, i, 1, WHITE);
        int y = 120 - 115 * cos(radians(i));
        if (i == 180) y -= 7;
        M5.Lcd.setCursor(160 + 115 * sin(radians(i)) - 5, y);
        if (i % 90 == 0) {  // 90°ごとにN、E、S、Wを書く
            M5.Lcd.print("NESW"[i / 90]);
        } else {
            M5.Lcd.print(i);
        }
    }
}

void readCurveSensor(double *curve) {
  float value = analogRead(CH_PIN);
  double dV = value * MAX_VOLTAGE / ANALOG_MAX;
  double dR = -1.0;
  if( 0.005 < (5.0 - dV) )
  {
    dR = 10 * 1000 * dV / (5.0 - dV);
  }
  curve[0] = dV;
  curve[1] = dR;
}

void bmm150_calibrate(uint32_t calibrate_time){ //bbm150 data calibrate.  bbm150数据校准
    uint32_t calibrate_timeout = 0;

    calibrate_timeout = millis() + calibrate_time;
    Serial.printf("Go calibrate, use %d ms \r\n", calibrate_time);  //The serial port outputs formatting characters.  串口输出格式化字符
    Serial.printf("running ...");

    while (calibrate_timeout > millis()){
        bmm150_read_mag_data(&dev); //read the magnetometer data from registers.  从寄存器读取磁力计数据
        if(dev.data.x){
            mag_min.x = (dev.data.x < mag_min.x) ? dev.data.x : mag_min.x;
            mag_max.x = (dev.data.x > mag_max.x) ? dev.data.x : mag_max.x;
        }
        if(dev.data.y){
            mag_max.y = (dev.data.y > mag_max.y) ? dev.data.y : mag_max.y;
            mag_min.y = (dev.data.y < mag_min.y) ? dev.data.y : mag_min.y;
        }
        if(dev.data.z){
            mag_min.z = (dev.data.z < mag_min.z) ? dev.data.z : mag_min.z;
            mag_max.z = (dev.data.z > mag_max.z) ? dev.data.z : mag_max.z;
        }
        delay(100);
    }

    mag_offset.x = (mag_max.x + mag_min.x) / 2;
    mag_offset.y = (mag_max.y + mag_min.y) / 2;
    mag_offset.z = (mag_max.z + mag_min.z) / 2;
    bmm150_offset_save();

    Serial.printf("\n calibrate finish ... \r\n");
    Serial.printf("mag_max.x: %.2f x_min: %.2f \t", mag_max.x, mag_min.x);
    Serial.printf("y_max: %.2f y_min: %.2f \t", mag_max.y, mag_min.y);
    Serial.printf("z_max: %.2f z_min: %.2f \r\n", mag_max.z, mag_min.z);
}

void handleRoot() {  // "/"をアクセスされたときの処理関数
    server.send(200, "text/plain", "hello from M5Stack!");
    M5.Lcd.println("accessed on \"/\"");
}

void get_sensor_data(float *mx, float *my, double *dV){
    double curve[2];
    readCurveSensor(curve);
    *dV = curve[0];

    bmm150_read_mag_data(&dev);
    *mx = dev.data.x - mag_offset.x;
    *my = dev.data.y - mag_offset.y;

    //Serial.printf("fs:%d\n x:%.2f mx:%.2f　y:%.2f my:%.2f　sp:%.2f V\n", flow_sensor_number, dev.data.x, *mx, dev.data.y, *my, *dV);

}

char mbuf[5000];
void handleEnv() { // "/env"をアクセスされたときの処理関数
    char buf[5000];  // HTMLを編集する文字配列        
    char buf2[400];

    float mx, my;
    double dv;
    get_sensor_data(&mx, &my, &dv);

    sprintf(buf2, "%.1f, %.1f, %.1f<br>", mx, my, dv);
    strcat(mbuf, buf2);

   // M5.Lcd.printf("Temp: %.1f, Humid: %.1f\r\n", temp, humid);
    sprintf(buf,  // HTMLに温度と湿度を埋め込む
        "<html>\
         <head>\
            <title>M5Stack EvnServer</title>\
            <meta http-equiv=\"Refresh\" content=\"5\">\
         <head>\
         <body>\
            <h1>M5Stack EnvServer</h1>\
            <p>mx,  my,  fspeed</p>\
            <p>%s</p>\
         </body>\
         </html>",
    mbuf);
    server.send(200, "text/html", buf);
}

void handleNotFound() {  // 存在しないファイルにアクセスされたときの処理関数
    server.send(404, "text/plain", "File Not Found\n\n");
    M5.Lcd.println("File Not Found");
}

int oldangle = 0;

void setup() {
    String config_ini;
    String ssid;
    String rola_id;
    String password;

    M5.begin();                                               // M5Stack の初期化
    Serial.begin(115200);                                     // シリアル接続の初期化

    M5.Lcd.setTextFont(4);                                    // テキストフォント：26ピクセルASCIIフォント
    M5.Lcd.setTextSize(1);                                    // テキストサイズ

    if (!SD.begin()) {                                        // SDカードの初期化
        Serial.println("Card failed, or not present");          // SDカードが未挿入の場合の出力
        //while (1);
    }
    else{
        Serial.println("microSD card initialized.");
    }
 
    /* ファイルオープン */
    File datFile = SD.open("/set/setup.ini");
    if( datFile ){
        Serial.println("File open successful");
        /* サイズ分ループ */
        while(datFile.available()){
            config_ini = config_ini + datFile.readString();
        }
        /* ID取得 */
        config_ini = config_ini.substring(config_ini.indexOf("#LoRaID\r\n") + 7);
        rola_id = config_ini.substring(0, config_ini.indexOf("\r\n"));
        datFile.close();
    } 
    else{
        Serial.println("File open error hello.txt");
        ssid = "W03_A8C83A9A1B29";
        password = "fmjt8rri8yq8657";
        //ssid = "aterm-644688-g";
        //password = "211dfba3d009c";
        //ssid = "Buffalo-G-76C0";
        //password = "d36tcnx64vsuh";
    }

    /* wifi初期化 */
    WiFi.begin(ssid.c_str(), password.c_str());// Wi-Fi APに接続する
    while (WiFi.status() != WL_CONNECTED) {  //  Wi-Fi AP接続待ち
        delay(500);
        M5.Lcd.print(".");
    }
    if (MDNS.begin("m5stack")) {
        Serial.println("MDNS responder started");
    }

    server.on("/", handleRoot);
    server.on("/env", handleEnv);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("HTTP server started");

    Wire.begin(21, 22, 400000UL); //Set the frequency of the SDA SCL.  设置SDA和SCL的频率

    img.setColorDepth(1);   //Set bits per pixel for colour.  设置色深为1
    img.setTextColor(TFT_WHITE);    //Set the font foreground colour (background is.  设置字体的前景色为TFT_WHITE
    img.createSprite(320, 240); //Create a sprite (bitmap) of defined width and height 创建一个指定宽度和高度的Sprite图
    img.setBitmapColor(TFT_WHITE, 0);   //Set the foreground and background colour.  设置位图的前景色和背景颜色

    if(bmm150_initialization() != BMM150_OK){
        img.fillSprite(0);  //Fill the whole sprite with defined colour.  用定义的颜色填充整个Sprite图
        img.drawCentreString("BMM150 init failed", 160, 110, 4);    //Use font 4 in (160,110)draw string.  使用字体4在(160,110)处绘制字符串
        img.pushSprite(0, 0);   //Push the sprite to the TFT at 0, 0.  将Sprite图打印在(0,0)处
        for(;;){
            delay(100); //delay 100ms.  延迟100ms
        }
    }

    bmm150_offset_load();

    drawGrid();

//    if (LoRaInit(flow_sensor_number) < 0) Serial.print("LoRa Error");

}


void loop() {
    M5.update();    //Read the press state of the key.  读取按键的状态

    float mx, my;
    double dv;
    get_sensor_data(&mx, &my, &dv);

    char text_string[100];
    bmm150_read_mag_data(&dev);
    float head_dir =
        atan2(dev.data.x - mag_offset.x, dev.data.y - mag_offset.y) * 180.0 /
        M_PI;
    Serial.printf("Magnetometer data, heading %.2f\n", head_dir);
    Serial.printf("MAG X : %.2f \t MAG Y : %.2f \t MAG Z : %.2f \n", dev.data.x,
                  dev.data.y, dev.data.z);
    Serial.printf("MID X : %.2f \t MID Y : %.2f \t MID Z : %.2f \n",
                  mag_offset.x, mag_offset.y, mag_offset.z);

    img.fillSprite(0);
    sprintf(text_string, "MAG X: %.2f", dev.data.x);
    img.drawString(text_string, 10, 20,
                   4);  // draw string with padding.  绘制带有填充的字符串
    sprintf(text_string, "MAG Y: %.2f", dev.data.y);
    img.drawString(text_string, 10, 50, 4);
    sprintf(text_string, "MAG Z: %.2f", dev.data.z);
    img.drawString(text_string, 10, 80, 4);
    sprintf(text_string, "HEAD Angle: %.2f", head_dir);
    img.drawString(text_string, 10, 110, 4);
    img.drawCentreString("Press BtnA enter calibrate", 160, 150, 4);
    img.pushSprite(0, 0);

    //int angle = (int)degrees(atan2(my, mx));//for mpu9
    int angle = (int)degrees(atan2(mx, my));//bmp150とmpu9はX軸が逆になっている。for bmm150
    drawLineByAngle(160, 120, 0, 85, oldangle, 1, BLACK);  // 古い線を消す
    drawLineByAngle(160, 120, 0, 85, oldangle - 180, 1, BLACK);
    drawLineByAngle(160, 120, 0, 85, angle, 1, RED);  // 新しい線の北側を描く
    drawLineByAngle(160, 120, 0, 85, angle - 180, 1, WHITE);  // 南側を描く

    oldangle = angle;

    // 画面表示
    M5.Lcd.setCursor(0,0);
    M5.Lcd.print("IP address: ");
    M5.Lcd.println(WiFi.localIP());
    M5.Lcd.printf("flow speed: %.2f V \n", dv);
        
    server.handleClient();

    if(M5.BtnA.wasPressed()){
        img.fillSprite(0);
        img.drawCentreString("Flip + rotate core calibration", 160, 110, 4);
        img.pushSprite(0, 0);
        bmm150_calibrate(10000);
        drawGrid();
    }

    delay(100);

}