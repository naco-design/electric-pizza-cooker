#include <Arduino.h>
#include <max6675.h>
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>

// 定数
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // リセットピンが接続されていない場合は-1

// SH1106ディスプレイオブジェクトの作成
Adafruit_SH1106 display(OLED_RESET);

// 温度センサークラス
class TemperatureSensor
{
public:
    MAX6675 thermocouple;
    TemperatureSensor(int clk, int cs, int doPin) : thermocouple(clk, cs, doPin) {}

    float readTemperature()
    {
        return thermocouple.readCelsius();
    }
};

// エラーチェッククラス
class ErrorChecker
{
public:
    bool checkError(float tempValue, const char *sensorName)
    {
        if (isnan(tempValue))
        {
            Serial.print("Error: ");
            Serial.print(sensorName);
            Serial.println(" TC_Open");
            return true;
        }
        return false;
    }

    bool checkOverTemp(float tempValue, float limit, const char *sensorName)
    {
        if (tempValue > limit)
        {
            Serial.print("Error: ");
            Serial.print(sensorName);
            Serial.println(" HiTEMP");
            return true;
        }
        return false;
    }
};

// RCフィルタクラス
class RCFilter
{
private:
    float RCfilter;

public:
    RCFilter(float filterValue) : RCfilter(filterValue) {}

    float applyFilter(float currentTemp, float prevFilteredTemp)
    {
        return RCfilter * prevFilteredTemp + (1 - RCfilter) * currentTemp;
    }
};

// PID制御クラス
class PIDController
{
private:
    PID myPID;
    double input, output, setpoint;
    float aggKp, aggKi, aggKd;
    float consKp, consKi, consKd;

public:
    PIDController(double *input, double *output, double *setpoint, float segTemp, float segSec, float segNSec, unsigned long interval)
        : myPID(input, output, setpoint, 0, 0, 0, DIRECT),
          input(*input), output(*output), setpoint(*setpoint)
    {
        aggKp = 1.2 / ((segTemp / segSec) * segNSec) * 999;
        aggKi = aggKp * ((interval / 1000) / (2 * segNSec));
        aggKd = aggKp * ((0.05 * segNSec) / (interval / 1000));

        consKp = aggKp * 2;
        consKi = aggKi * 2;
        consKd = aggKd;

        myPID.SetTunings(consKp, consKi, consKd);
        myPID.SetMode(AUTOMATIC);
    }

    void setTunings(double gap)
    {
        if (gap < 10)
        {
            myPID.SetTunings(consKp, consKi, consKd);
        }
        else
        {
            myPID.SetTunings(aggKp, aggKi, aggKd);
        }
    }

    void compute()
    {
        myPID.Compute();
    }

    double getOutput()
    {
        return output;
    }
};

// SSR制御クラス
class SSRController
{
public:
    int pin;
    SSRController(int outputPin) : pin(outputPin)
    {
        pinMode(pin, OUTPUT);
    }

    void write(double value)
    {
        analogWrite(pin, value);
    }

    void stop()
    {
        analogWrite(pin, 0);
    }
};

// ロータリーエンコーダとスイッチを処理するクラス
class EncoderHandler
{
private:
    const int clkPin, dtPin, swPin;
    int lastClkState, currentClkState;
    bool inEditMode;                        // 編集モードか選択モードかを判別する
    int selectedItem;                       // 0: targetTEMP_Up, 1: targetTEMP_Lw
    unsigned long lastEncoderMillis;        // デバウンス用のタイムスタンプ
    const unsigned long debounceDelay = 50; // デバウンス時間（ミリ秒）
    int currentPresetIndex;                 // 選択中のプリセット

public:
    EncoderHandler(int clk, int dt, int sw)
        : clkPin(clk), dtPin(dt), swPin(sw), lastClkState(LOW), inEditMode(false), selectedItem(0), lastEncoderMillis(0), currentPresetIndex(-1)
    {
        pinMode(clkPin, INPUT);
        pinMode(dtPin, INPUT);
        pinMode(swPin, INPUT_PULLUP); // スイッチはプルアップ抵抗を使用
    }

    // エンコーダの状態を確認し、選択または編集する
    void update(int &targetUp, int &targetLw, int presetsUp[], int presetsLw[], const char *presetNames[])
    {
        // 現在の時間を取得
        unsigned long currentMillis = millis();

        // デバウンス処理: 前回の回転信号処理から一定時間が経過していなければ無視
        if (currentMillis - lastEncoderMillis > debounceDelay)
        {
            currentClkState = digitalRead(clkPin);
            if (currentClkState != lastClkState && currentClkState == HIGH)
            {
                bool direction = digitalRead(dtPin) != currentClkState; // 回転方向を判定

                if (inEditMode)
                {
                    // 編集モードでは数値を増減する
                    if (selectedItem == 0) // targetUpを編集
                    {
                        targetUp += direction ? 5 : -5;
                        targetUp = constrain(targetUp, 40, 500); // 増減後に範囲を制限
                    }
                    else if (selectedItem == 1) // targetLwを編集
                    {
                        targetLw += direction ? 5 : -5;
                        targetLw = constrain(targetLw, 40, 500); // 増減後に範囲を制限
                    }
                    else if (selectedItem == 2) // プリセットを選択
                    {
                        currentPresetIndex += direction ? 1 : -1;
                        currentPresetIndex = (currentPresetIndex + 4) % 4; // 0から2の範囲に制限
                        targetUp = presetsUp[currentPresetIndex];
                        targetLw = presetsLw[currentPresetIndex];
                    }
                }
                else
                {
                    // 選択モードでは項目を切り替える
                    selectedItem = (selectedItem + (direction ? 1 : -1) + 3) % 3; // 項目切り替えを範囲内に制限
                }

                // 回転信号の処理時間を更新
                lastEncoderMillis = currentMillis;
            }
            lastClkState = currentClkState;
        }

        // スイッチが押されたか確認
        if (digitalRead(swPin) == LOW)
        {
            delay(50); // スイッチのデバウンス処理
            if (digitalRead(swPin) == LOW)
            {
                // 編集モードをトグル
                inEditMode = !inEditMode;
                delay(500); // 押し下げ待ち
            }
        }
    }

    // 選択中の項目を返す
    int getSelectedItem() const
    {
        return selectedItem;
    }

    int getCurrentPresetIndex() const
    {
        return currentPresetIndex;
    }

    // 編集モードかどうか
    bool isEditMode() const
    {
        return inEditMode;
    }
};

// ピンアサイン
const int thermoDO = 14;   // MAX6675 SO
const int thermoCLK = 15;  // MAX6675 SCK
const int thermoCS1 = A1;  // 上プレートTC MAX6675CS
const int thermoCS2 = A0;  // 上ヒーターTC MAX6675CS
const int thermoCS3 = A3;  // 下プレートTC MAX6675CS
const int thermoCS4 = A2;  // 下ヒーターTC MAX6675CS
const int SSR1_OUTPUT = 5; // 上ヒーターSSR
const int SSR2_OUTPUT = 6; // 下ヒーターSSR
const int encoderCLK = 7;  // ロータリエンコーダKY-040 CLK
const int encoderDT = 8;   // ロータリエンコーダKY-040 DT
const int encoderSW = 4;   // ロータリエンコーダKY-040 SW
// const int OLED_SDA = 2;    // I2C SDAピン
// const int OLED_SCL = 3;    // I2C SCLピン

// 調整パラメーター
int targetTEMP_Up = 485;         // 上プレート設定温度 窯の天井の温度 ：約485度
int targetTEMP_Lw = 400;         // 下プレート設定温度 炉床の温度 ：約380～430度
const int LimitHeaterTEMP = 780; // ヒーターリミット
unsigned long TCinterval = 1000; // TC計測間隔
float RCfilterValue = 0.8;       // RCフィルター係数

// ステップ応答法によるPID制御パラメータ（上プレート用）
float SegTemp_Up = 23; // x℃上昇するのに
float SegSec_Up = 60;  // かかった秒数
float SegNSec_Up = 30; // 反応までにかかる秒数

// ステップ応答法によるPID制御パラメータ（下プレート用）
float SegTemp_Lw = 22;
float SegSec_Lw = 60;
float SegNSec_Lw = 30;

// プリセットの定義
int presetsUp[4] = {485, 330, 300, 220};                                   // 上プレートのプリセット値
int presetsLw[4] = {400, 270, 300, 200};                                   // 下プレートのプリセット値
const char *presetNames[4] = {"Napoletana", "Romana", "calzone", "220/200"}; // プリセットの名前
int currentPresetIndex = -1;                                               // 現在選択されているプリセットインデックス (-1は手動設定)

// Ziegler-Nichols法用の暫定パラメータ
double Ku_Up = 4.0, Tu_Up = 20.0; // 上ヒーター用のZiegler-Nicholsパラメータ
double Ku_Lw = 4.0, Tu_Lw = 20.0; // 下ヒーター用のZiegler-Nicholsパラメータ

double Setpoint_Up, Input_Up, Output_Up, Setpoint_Lw, Input_Lw, Output_Lw;

// 各クラスのインスタンス作成
TemperatureSensor sensor1(thermoCLK, thermoCS1, thermoDO); // 上プレート
TemperatureSensor sensor2(thermoCLK, thermoCS2, thermoDO); // 上ヒーター
TemperatureSensor sensor3(thermoCLK, thermoCS3, thermoDO); // 下プレート
TemperatureSensor sensor4(thermoCLK, thermoCS4, thermoDO); // 下ヒーター

ErrorChecker errorChecker;
RCFilter rcFilter(RCfilterValue);
PIDController UpPID(&Input_Up, &Output_Up, &Setpoint_Up, SegTemp_Up, SegSec_Up, SegNSec_Up, TCinterval);
PIDController LwPID(&Input_Lw, &Output_Lw, &Setpoint_Lw, SegTemp_Lw, SegSec_Lw, SegNSec_Lw, TCinterval);

SSRController ssrUp(SSR1_OUTPUT);
SSRController ssrLw(SSR2_OUTPUT);

EncoderHandler encoder(encoderCLK, encoderDT, encoderSW);

unsigned long TCpreviousMillis = 0;
bool ErrorMode_Up = false;
bool ErrorMode_Lw = false;
float TC_UpPlateRC, TC_LwPlateRC;

// LCDに温度を表示する関数
void displayTemperature()
{
    display.clearDisplay();      // 画面をクリア
    display.setTextSize(1);      // テキストサイズを設定
    display.setTextColor(WHITE); // テキスト色を白に設定

    display.setCursor(0, 0); // カーソルを設定
    display.print("UP TGT: ");
    display.print(targetTEMP_Up); // 上プレート目標温度の表示
    display.print(" C");
    if (encoder.getSelectedItem() == 0)
    { // 選択中の項目がtargetTEMP_Upの場合
        if (encoder.isEditMode())
        {
            display.print(" <edit"); // 編集モード中なら "<" を表示
        }
        else
        {
            display.print(" *"); // 選択中だけど編集モードではないときに "*" を表示
        }
    }

    display.setCursor(0, 10); // 2行目に移動
    display.print("UP PLT: ");
    display.print(TC_UpPlateRC); // 上プレート実温度の表示
    display.print(" C");

    display.setCursor(0, 20); // 3行目に移動
    display.print("LW TGT: ");
    display.print(targetTEMP_Lw); // 下プレート目標温度の表示
    display.print(" C");
    if (encoder.getSelectedItem() == 1)
    { // 選択中の項目がtargetTEMP_Lwの場合
        if (encoder.isEditMode())
        {
            display.print(" <edit"); // 編集モード中なら "<" を表示
        }
        else
        {
            display.print(" *"); // 選択中だけど編集モードではないときに "*" を表示
        }
    }

    display.setCursor(0, 30); // 4行目に移動
    display.print("LW PLT: ");
    display.print(TC_LwPlateRC); // 下プレート実温度の表示
    display.print(" C");

    // プリセット名の表示
    display.setCursor(0, 40);
    if (encoder.getSelectedItem() == 2)
    {
        if (encoder.getCurrentPresetIndex() != -1)
        {
            display.print(presetNames[encoder.getCurrentPresetIndex()]); // プリセット名を表示
        }
        else
        {
            display.print("Preset");
        }
        if (encoder.isEditMode())
        {
            display.print(" <"); // 編集モード中なら "<" を表示
        }
        else
        {
            display.print(" *"); // 選択中だけど編集モードではないときに "*" を表示
        }
    }

    display.display(); // 画面に反映
}

void setup()
{
    Serial.begin(9600);
    Serial.println("Pizza Time");

    // SH1106の初期化
    display.begin(SH1106_SWITCHCAPVCC, 0x3C); // 初期化のみ行う
    if (display.width() == 0)
    { // 初期化が失敗した場合
        Serial.println(F("OLED allocation failed"));
        while (1)
            ; // 無限ループで停止
    }
    display.clearDisplay();
    display.display();

    Setpoint_Up = targetTEMP_Up;
    Setpoint_Lw = targetTEMP_Lw;

    delay(500);
    TC_UpPlateRC = sensor1.readTemperature(); // 初期化
    TC_LwPlateRC = sensor3.readTemperature();
    delay(1000);
}

void loop()
{
    unsigned long currentMillis = millis();

    if ((currentMillis - TCpreviousMillis) > (TCinterval))
    {
        TCpreviousMillis = currentMillis;

        float TC_UpPlate = sensor1.readTemperature();
        float TC_UpHeater = sensor2.readTemperature();
        float TC_LwPlate = sensor3.readTemperature();
        float TC_LwHeater = sensor4.readTemperature();

        // エラーチェック
        ErrorMode_Up = errorChecker.checkError(TC_UpPlate, "TC_UpPlate") ||
                       errorChecker.checkError(TC_UpHeater, "TC_UpHeater") ||
                       errorChecker.checkOverTemp(TC_UpPlate, targetTEMP_Up + 50, "TC_UpPlate") ||
                       errorChecker.checkOverTemp(TC_UpHeater, LimitHeaterTEMP, "TC_UpHeater");
        ErrorMode_Lw = errorChecker.checkError(TC_LwPlate, "TC_LwPlate") ||
                       errorChecker.checkError(TC_LwHeater, "TC_LwHeater") ||
                       errorChecker.checkOverTemp(TC_LwPlate, targetTEMP_Lw + 50, "TC_LwPlate") ||
                       errorChecker.checkOverTemp(TC_LwHeater, LimitHeaterTEMP, "TC_LwHeater");

        // フィルタ処理
        TC_UpPlateRC = rcFilter.applyFilter(TC_UpPlate, TC_UpPlateRC);
        TC_LwPlateRC = rcFilter.applyFilter(TC_LwPlate, TC_LwPlateRC);

        // PID計算
        Input_Up = TC_UpPlateRC;
        Input_Lw = TC_LwPlateRC;
        UpPID.setTunings(abs(Setpoint_Up - Input_Up));
        LwPID.setTunings(abs(Setpoint_Lw - Input_Lw));

        UpPID.compute();
        LwPID.compute();

        // SSR制御
        if (ErrorMode_Up)
        {
            ssrUp.stop();
        }
        else
        {
            ssrUp.write(Output_Up);
        }
        if (ErrorMode_Lw)
        {
            ssrLw.stop();
        }
        else
        {
            ssrLw.write(Output_Lw);
        }

        // 温度の表示
        displayTemperature();

        // TC温度を表示 (Teleplot)
        Serial.print(">LimitHeaterTEMP:");
        Serial.println(LimitHeaterTEMP);
        Serial.print(">targetTEMP_Up:");
        Serial.println(targetTEMP_Up);
        Serial.print(">targetTEMP_Lw:");
        Serial.println(targetTEMP_Lw);
        Serial.print(">UpPlate:");
        Serial.println(TC_UpPlateRC);
        Serial.print(">LwPlate:");
        Serial.println(TC_LwPlateRC);
        Serial.print(">TC_UpHeater:");
        Serial.println(TC_UpHeater);
        Serial.print(">TC_LwHeater:");
        Serial.println(TC_LwHeater);
        Serial.print(">Output_Up:");
        Serial.println(Output_Up);
        Serial.print(">Output_Lw:");
        Serial.println(Output_Lw);
    }
    // ロータリーエンコーダを使用して目標温度を調整
    encoder.update(targetTEMP_Up, targetTEMP_Lw, presetsUp, presetsLw, presetNames);
}