#include <Arduino.h>
#include <BleGamepad.h>
#include <esp_now.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <math.h>

#define QuarterPi 0.78539816339
#define HalfPi 1.57079632679
#define ThreeQuarterPi 2.35619449019

unsigned long A_time, TimeLastSent;
int value[6];
bool Send = 0;
bool ShouldArmaOn = false;
byte FreioCntD, FreioCntE;
esp_err_t SendStatus;
// bool ButtonPressed;
unsigned long rtt;
bool buttons_state[18];
bool buttons_last_state[18];

// uint8_t broadcastAddress[3][6] = {{0xCC, 0x50, 0xE3, 0x56, 0xAD, 0xF4}, // Alley
//                                   {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
//                                   {0x60, 0x01, 0x94, 0x0F, 0x8A, 0xD3}}; // 60:01:94:0F:8A:D3
//  uint8_t broadcastAddress[][6] = {};
uint8_t broadcastAddress[][6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast
// uint8_t broadcastAddress[][6] = {0x48, 0x55, 0x19, 0x00, 0x4A, 0xAF}; // D1?
unsigned long send_time = 0;
unsigned long CH_switch_time = 0;
bool Send_status = false;
esp_now_peer_info_t peerInfo;
#define Channel 0

// Velocidade da reprodução dos leds
unsigned int period = 40;
// Cor dos leds frontais
byte HUE = 0;
/// Valor minimo da velocidade que faz o robo se mover
byte minVel = 0;

/// Padrão de piscada dos leds
bool Effect[4][19] = {
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // R1
    {},                                                        // B1
    {},                                                        // R2
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0}, // B2
};

/// Estrutura de comunicação EspNow
typedef struct struct_message
{
    /// Valor entre -255 e 255
    signed int motor_e;
    /// Valor entre -255 e 255
    signed int motor_d;
    /// Valor entre 1000ms e 2000ms
    int arma_vel;
    /// Primeiro indice é o led, o segundo são os valores de RGB (0-255 cada)
    byte led[5][3];
    /// 1 para ativo 0 caso contrario
    bool FreioE;
    bool FreioD;
    /// Canais para qual será migrado
    byte ChannelNext[2];
} struct_message;

struct_message Controle;
struct_message LastControle;

int calRX, calRY, calLX, calLY;

#define Triangulo 21
#define Quadrado 19
#define Bolinha 18
#define Xhis 5
#define Up 25
#define Down 27
#define Right 14
#define Left 26
#define R1 23
#define R2 22
#define L1 39
#define L2 36
#define Start 16
#define Select 12
#define Analog 17

#define RX 35
#define RY 34
#define RT 4
#define LX 32
#define LY 33
#define LT 15

#define Led 2

#define numOfButtons 18
// #define numOfHatSwitches 2

BleGamepad bleGamepad;
BleGamepadConfiguration bleGamepadConfig;
bool ControllerMode;

int Average(uint8_t pin)
{
    int Read = 0;
    int i = 0;
    for (i = 0; i < 10; i++)
        Read += analogRead(pin);
    return Read / i;
}

int analogGet(uint8_t pin, int cal)
{
    int Read = Average(pin);
    // Serial.println(Read - cal);
    return (Read - cal) < 2048 ? map(Read - cal, -cal, 2047, 32767, 16384) : map(Read - cal, 2048, 4095 - cal, 16384, 0);
}

void Calibrar()
{
    // calRX = analogRead(RX) - 2048;
    // calRY = analogRead(RY) - 2048;
    // calLX = analogRead(LX) - 2048;
    // calLY = analogRead(LY) - 2048;

    calRX = Average(RX) - 2048;
    calRY = Average(RY) - 2048;
    calLX = Average(LX) - 2048;
    calLY = Average(LY) - 2048;

    digitalWrite(Led, LOW);
    delay(100);
    digitalWrite(Led, HIGH);
}

void Print()
{
    if (!ControllerMode)
    {
        Serial.print(digitalRead(Triangulo));
        Serial.print(" ");
        Serial.print(digitalRead(Quadrado));
        Serial.print(" ");
        Serial.print(digitalRead(Bolinha));
        Serial.print(" ");
        Serial.print(digitalRead(Xhis));
        Serial.print(" ");
        Serial.print(digitalRead(Up));
        Serial.print(" ");
        Serial.print(digitalRead(Down));
        Serial.print(" ");
        Serial.print(digitalRead(Right));
        Serial.print(" ");
        Serial.print(digitalRead(Left));
        Serial.print(" ");
        Serial.print(digitalRead(R1));
        Serial.print(" ");
        Serial.print(digitalRead(R2));
        Serial.print(" ");
        Serial.print(digitalRead(L1));
        Serial.print(" ");
        Serial.print(digitalRead(L2));
        Serial.print(" ");
        Serial.print(digitalRead(Start));
        Serial.print(" ");
        Serial.print(digitalRead(Select));
        Serial.print(" ");
        Serial.print(analogRead(RX));
        Serial.print(" ");
        Serial.print(analogRead(RY));
        Serial.print(" ");
        Serial.print(analogRead(LX));
        Serial.print(" ");
        Serial.print(analogRead(LY));
        Serial.print(" ");
        Serial.println();
    }
    else
    {
        Serial.print(Controle.motor_e);
        Serial.print("  ");
        Serial.print(Controle.motor_d);
        Serial.print("  ");
        Serial.print(Controle.FreioD);
        Serial.print("  ");
        Serial.print(Controle.FreioE);
        Serial.print("  ");
        Serial.println(Controle.arma_vel);
    }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    Serial.print(status == ESP_NOW_SEND_SUCCESS ? "1" : "0");
    // if (status != ESP_NOW_SEND_SUCCESS && millis() - TimeLastSent < 15)
    //   esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
    Serial.println();
}
/// Calibrar valores recebidos do controle
void CalibrateValues()
{
    value[0] = constrain(map(analogGet(LX, calRX), 0, 32767, -255, 255), -255, 255); // X
    value[1] = constrain(map(analogGet(LY, calRY), 0, 32767, -255, 255), -255, 255); // Y
    value[2] = constrain(map(analogGet(RY, calLY), 0, 32767, 0, 255), 0, 255);       // Arma
    value[3] = constrain(map(analogGet(RX, calLX), 0, 32767, 0, 255), -255, 255);    // L arma
    // value[4] = constrain(map(value[4], 20, 25, 0, 255), 0, 255);                     // Cima/baixo
}
/// Salva valores convertidos do motor e corrige caso robo capote
void Save2send(float Angle, int vel)
{
    if (value[4]) // Testa se robo capotou
    {
        Controle.motor_d = vel * sin(HalfPi * cos(Angle - ThreeQuarterPi));
        Controle.motor_e = vel * sin(HalfPi * cos(Angle - QuarterPi));
    }
    else
    {
        Controle.motor_e = -vel * sin(HalfPi * cos(Angle - ThreeQuarterPi));
        Controle.motor_d = -vel * sin(HalfPi * cos(Angle - QuarterPi));
    }
}
/// Converter valores para direção do motor
void ConverterMotores(signed int X, signed int Y)
{
    X = X == 0 ? 1 : X;
    float Angle = atan((float)Y / (float)X);
    if (X < 0)
        Angle += Y > 0 ? PI : -PI;

    // Module = sqrt(X * X + Y * Y);
    int Module = X / cos(Angle);
    // int Vel = Module * Module / 255;

    int Max_Vel = (abs(X) > abs(Y)) ? 255 / abs(cos(Angle)) : 255 / abs(sin(Angle));
    // int vel = constrain(255 * Module / Max_Vel, 0, 255);
    int vel = constrain(Module * Module / Max_Vel, 0, 255);
    Save2send(Angle, vel);
}
/// DeadZone e Limite de velocidade
void LimitarMotores()
{
    if (abs(Controle.motor_d) > 30)
    {
        if (Controle.motor_d > 0)
            Controle.motor_d = map(Controle.motor_d, 0, 255, minVel, 255);
        else
            Controle.motor_d = map(Controle.motor_d, 0, -255, -minVel, -255);
    }
    else
        Controle.motor_d = 0;

    if (abs(Controle.motor_e) > 30)
    {
        if (Controle.motor_e > 0)
            Controle.motor_e = map(Controle.motor_e, 0, 255, minVel, 255);
        else
            Controle.motor_e = map(Controle.motor_e, 0, -255, -minVel, -255);
    }
    else
        Controle.motor_e = 0;
}

float fract(float x) { return x - int(x); }
float mix(float a, float b, float t) { return a + (b - a) * t; }
/// Converter valores de HSV para RGB
void hsv2rgb(float h, float s, float b)
{
    // h = hue; s = saturation; b = brightness
    Controle.led[2][0] = s * mix(1.0, constrain(abs(fract(h + 1.0) * 6.0 - 3.0) - 1.0, 0.0, 1.0), b);
    Controle.led[2][1] = s * mix(1.0, constrain(abs(fract(h + 0.6666666) * 6.0 - 3.0) - 1.0, 0.0, 1.0), b);
    Controle.led[2][2] = s * mix(1.0, constrain(abs(fract(h + 0.3333333) * 6.0 - 3.0) - 1.0, 0.0, 1.0), b);

    Controle.led[3][0] = Controle.led[2][0];
    Controle.led[3][1] = Controle.led[2][1];
    Controle.led[3][2] = Controle.led[2][2];

    Controle.led[4][0] = Controle.led[2][0];
    Controle.led[4][1] = Controle.led[2][1];
    Controle.led[4][2] = Controle.led[2][2];
}
/// Blink leds padrão sirene
void PoliceEffectLed()
{
    unsigned int index = (millis() - A_time) / period;

    Controle.led[0][0] = 255 * Effect[0][index];
    Controle.led[0][2] = 255 * Effect[1][index];
    Controle.led[1][0] = 255 * Effect[2][index];
    Controle.led[1][2] = 255 * Effect[3][index];

    if (index >= sizeof(Effect[0]))
        A_time = millis();

    HUE++;
    hsv2rgb(170, 255, 255);
}
/// Limitador da velocidade da arma
void ConstrainArma()
{
    if (ShouldArmaOn)
    {
        if (value[4])
            Controle.arma_vel = constrain(map(value[2], 0, 255, 1500, 2000), 1500, 2000);
        else
            Controle.arma_vel = constrain(map(value[2], 0, 255, 1500, 1000), 1000, 1500);

        if (abs(Controle.arma_vel - 1500) < 50)
            Controle.arma_vel = 1500;
    }
    else
        Controle.arma_vel = 1500;
}
/// Identifica entrada na DeadZone para freiar brevemente
void Freio()
{
    if (!Controle.motor_d)
    {                                                 // É pra parar?
        if (FreioCntD)                                // Ja começou a freiar?
            if (FreioCntD <= 10)                      // É pra continuar freiando?
                FreioCntD++;                          // Freia e aumenta a contagem
            else                                      //
                FreioCntD = 0;                        // Para e freiar
        else if (LastControle.motor_d)                // Ainda nao começou a freiar, mas deve?
            FreioCntD = 1;                            // Freia e começa a contagem
    }                                                 //
    else                                              //
        FreioCntD = 0;                                // Entao é pra continuar andando
    Controle.FreioD = (FreioCntD > 0 ? true : false); // Aplica o freio

    if (!Controle.motor_e)
    {
        if (FreioCntE)
            if (FreioCntE <= 10)
                FreioCntE++;
            else
                FreioCntE = 0;
        else if (LastControle.motor_e)
            FreioCntE = 1;
    }
    else
        FreioCntE = 0;
    Controle.FreioE = (FreioCntE > 0 ? true : false);
}
/// Envia os movimentos e inicia a contagem de tempo de envio
void SendMoves()
{
    // Serial.println(broadcastAddress[0][0], HEX);
    Send_status = false;
    send_time = micros();
    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
}
/// Troca de o canal de comunicação com o robo
void SwitchChannel()
{
    Controle.ChannelNext[0] = Controle.ChannelNext[1];
    Controle.ChannelNext[1] = random(1, 13);
    Send_status = false;
    CH_switch_time = millis();
    while (Send_status == false)
    {
        Serial.print("Esperando: ");
        Serial.println(Send_status);
        esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
        if (millis() - CH_switch_time > 50)
        {
            Controle.ChannelNext[0] = 0;
            break;
        }
        delay(5);
    }
    Serial.println("Saindo do loop");

    // Confirmar Recepção
    // CH_swithch_time = millis();
    WiFi.softAP(" ", NULL, Controle.ChannelNext[0], 1, 0, false);
    WiFi.disconnect();
    peerInfo.channel = Controle.ChannelNext[0];
    esp_now_mod_peer(&peerInfo);
    Serial.print("Tempo de troca (ms): ");
    Serial.println(millis() - CH_switch_time);
    Serial.println();
    Serial.print("canal: ");
    Serial.println(peerInfo.channel);
    send_time = micros();
}
/// Testa se algum botão foi atertado
void TestButtons()
{
    for (int i = 1; i < 18; i++)
        buttons_last_state[i] = buttons_state[i];

    buttons_state[1] = digitalRead(Triangulo);
    buttons_state[2] = digitalRead(Quadrado);
    buttons_state[3] = digitalRead(Bolinha);
    buttons_state[4] = digitalRead(Xhis);
    buttons_state[5] = digitalRead(Up);
    buttons_state[6] = digitalRead(Down);
    buttons_state[7] = digitalRead(Right);
    buttons_state[8] = digitalRead(Left);
    buttons_state[9] = digitalRead(R1);
    buttons_state[10] = digitalRead(R2);
    buttons_state[11] = digitalRead(L1);
    buttons_state[12] = digitalRead(L2);
    buttons_state[13] = digitalRead(LT);
    buttons_state[14] = digitalRead(RT);
    buttons_state[15] = digitalRead(Start);
    buttons_state[16] = digitalRead(Select);
    buttons_state[17] = digitalRead(Analog);

    int i;
    for (i = 1; i < 18; i++)
        if (buttons_state[i] && !buttons_last_state[i])
            break; // Praticamente nunca vai acontecer de apertar dois botões tão sincronizados

    switch (i)
    {
    case 1:
        value[4] = !value[4];
        break;
    case 2:
        SwitchChannel();
        break;
    case 3:
        ShouldArmaOn = !ShouldArmaOn;
        break;
    }
}
/// Callback da confirmação de envio
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int data_len)
{
    rtt = micros() - send_time;
    // Serial.print("RTT: ");
    //  Serial.println(rtt);
    Send_status = true;

    if (broadcastAddress[0][0] == 0xFF && broadcastAddress[0][1] == 0xFF)
    {
        esp_now_del_peer(broadcastAddress[0]);
        memcpy(peerInfo.peer_addr, mac, 6);
        memcpy(broadcastAddress[0], mac, 6);
        //  Add peer
        if (esp_now_add_peer(&peerInfo) != ESP_OK)
        {
            Serial.println("Failed to add peer");
            return;
        }

        switch (broadcastAddress[0][0])
        {
        case 0xCC: // Alley
            minVel = 70;
            break;
        case 0x60: // Cuca
            minVel = 40;
            break;
        case 0x48: // Tecna
            minVel = 25;
            break;
        }
    }
}

void setup()
{
    pinMode(Led, OUTPUT);
    pinMode(L2, INPUT_PULLDOWN);
    digitalWrite(Led, HIGH);
    delay(50);
    ControllerMode = (analogRead(L2) < 3500) ? false : true;
    delay(50);
    digitalWrite(Led, LOW);

    Serial.begin(115200);

    if (!ControllerMode)
    {
        Serial.println("Starting BLE work!");
        bleGamepadConfig.setIncludeStart(true);
        bleGamepadConfig.setIncludeSelect(true);

        bleGamepadConfig.setAutoReport(false);
        bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD); // CONTROLLER_TYPE_JOYSTICK, CONTROLLER_TYPE_GAMEPAD (DEFAULT), CONTROLLER_TYPE_MULTI_AXIS
        bleGamepadConfig.setButtonCount(numOfButtons);
        // bleGamepadConfig.setHatSwitchCount(numOfHatSwitches);
        bleGamepadConfig.setVid(0xe502);
        bleGamepadConfig.setPid(0xabcd);
        // Some non-Windows operating systems and web based gamepad testers don't like min axis set below 0, so 0 is set by default
        // bleGamepadConfig.setAxesMin(0x8001); // -32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
        bleGamepadConfig.setAxesMin(0x0000); // 0 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
        bleGamepadConfig.setAxesMax(0x7FFF); // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
        bleGamepad.begin(&bleGamepadConfig); // Simulation controls, special buttons and hats 2/3/4 are disabled by default
        // changing bleGamepadConfig after the begin function has no effect, unless you call the begin function again
        bleGamepad.deviceName = "Controle ESP32";
        bleGamepad.deviceManufacturer = "Felipe Lenschow";
    }
    else
    {
        Serial.println("Starting WiFi work!");

        WiFi.softAP("Len", NULL, Channel, 0, 1, false);
        WiFi.disconnect();
        WiFi.mode(WIFI_STA);
        if (esp_now_init() != 0)
            Serial.println("ESPNow Init Failed");
        esp_now_register_recv_cb(OnDataRecv);

        // Register peer
        memcpy(peerInfo.peer_addr, broadcastAddress[0], 6);
        peerInfo.channel = Channel;
        peerInfo.encrypt = false;

        // Add peer
        if (esp_now_add_peer(&peerInfo) != ESP_OK)
        {
            Serial.println("Failed to add peer");
            return;
        }

        WiFi.setSleep(false);
        Controle.ChannelNext[0] = Channel;
        Controle.ChannelNext[1] = random(1, 13);
    }

    pinMode(Triangulo, INPUT);
    pinMode(Quadrado, INPUT_PULLDOWN);
    pinMode(Bolinha, INPUT_PULLDOWN);
    pinMode(Xhis, INPUT_PULLDOWN);
    pinMode(Up, INPUT_PULLDOWN);
    pinMode(Down, INPUT_PULLDOWN);
    pinMode(Right, INPUT_PULLDOWN);
    pinMode(Left, INPUT_PULLDOWN);
    pinMode(R1, INPUT_PULLDOWN);
    pinMode(R2, INPUT_PULLDOWN);
    pinMode(L1, INPUT_PULLDOWN);
    pinMode(L2, INPUT_PULLDOWN);
    pinMode(Start, INPUT_PULLDOWN);
    pinMode(Select, INPUT_PULLDOWN);
    pinMode(Analog, INPUT_PULLDOWN);
    pinMode(RX, INPUT);
    pinMode(RY, INPUT);
    pinMode(RT, INPUT);
    pinMode(LX, INPUT);
    pinMode(LY, INPUT);
    pinMode(LT, INPUT);

    Calibrar();
}

void loop()
{
    if (!ControllerMode)
    {
        bleGamepad.isConnected() ? digitalWrite(Led, HIGH) : digitalWrite(Led, LOW);
        (digitalRead(Analog)) ? Calibrar() : bleGamepad.release(17);

        (digitalRead(Triangulo)) ? bleGamepad.press(1) : bleGamepad.release(1);
        (digitalRead(Quadrado)) ? bleGamepad.press(2) : bleGamepad.release(2);
        (digitalRead(Bolinha)) ? bleGamepad.press(3) : bleGamepad.release(3);
        (digitalRead(Xhis)) ? bleGamepad.press(4) : bleGamepad.release(4);
        (digitalRead(Up)) ? bleGamepad.press(5) : bleGamepad.release(5);
        (digitalRead(Down)) ? bleGamepad.press(6) : bleGamepad.release(6);
        (digitalRead(Right)) ? bleGamepad.press(7) : bleGamepad.release(7);
        (digitalRead(Left)) ? bleGamepad.press(8) : bleGamepad.release(8);
        (digitalRead(R1)) ? bleGamepad.press(9) : bleGamepad.release(9);
        (digitalRead(R2)) ? bleGamepad.press(10) : bleGamepad.release(10);
        (digitalRead(L1)) ? bleGamepad.press(11) : bleGamepad.release(11);
        (digitalRead(L2)) ? bleGamepad.press(12) : bleGamepad.release(12);
        (!digitalRead(LT)) ? bleGamepad.press(13) : bleGamepad.release(13);
        (!digitalRead(RT)) ? bleGamepad.press(14) : bleGamepad.release(14);
        (digitalRead(Start)) ? bleGamepad.press(15) : bleGamepad.release(15);
        (digitalRead(Select)) ? bleGamepad.press(16) : bleGamepad.release(16);
        (digitalRead(Analog)) ? bleGamepad.press(17) : bleGamepad.release(17);

        bleGamepad.setAxes(analogGet(LX, calLX), analogGet(LY, calLY), 0, 0, analogGet(RX, calRX), analogGet(RY, calRY));

        bleGamepad.sendReport();
        delay(10);
        // Print();
    }
    else
    {
        digitalWrite(Led, !digitalRead(Led));
        if (digitalRead(Analog))
            Calibrar();
        LastControle = Controle;
        CalibrateValues();
        ConverterMotores(value[0], value[1]);
        LimitarMotores();
        Freio();
        ConstrainArma();
        PoliceEffectLed();

        TestButtons();

        SendMoves();

        while (Send_status == false)
            if (micros() - send_time > 50000)
            {
                digitalWrite(Led, HIGH);
                // delay(5000); //                      <-----------------------------------------------------------------
                //  Switch channel ou fail safe
                //SwitchChannel();
                break;
            }

        delay(10);
        // Print();
    }
}
