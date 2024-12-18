#include <mbed.h>
#include "C610.hpp"

CAN can{PA_11, PA_12, (int)1e6};
C610Array c610{};

// PID制御のゲイン
constexpr float kp = 1.0;   // 比例ゲイン
constexpr float ki = 0.0;   // 積分ゲイン
constexpr float kd = 0.03; // 微分ゲイン
constexpr float dt_sec = 0.005f; // 制御周期（秒）

constexpr int16_t target_rpm = 5000; // 目標RPM

int main() {
    const int motor_id = 1; // 制御するモーターのID（1～8）

    float integral_error = 0.0f; // 積分誤差
    int16_t previous_error = 0;  // 前回の誤差
    float derivative_error_filtered = 0.0f; // フィルタリング後の微分誤差
    constexpr float alpha = 0.1f; // 微分フィルタの係数（低いほどスムーズ）

    bool reached_target = false; // 目標RPM到達フラグ

    while (1) {
        auto now = HighResClock::now();
        static auto pre = now;
        auto dt = now - pre;

        // CANメッセージを読み取り
        if (CANMessage msg; can.read(msg)) {
            c610.parse_packet(msg);
        }

        // 10msごとに制御
        if (dt > 5ms) {
            pre = now;

            auto& motor = c610[motor_id - 1]; // 対応するモーターを取得
            int16_t current_rpm = motor.get_rpm(); // 現在のRPMを取得

            // RPMの誤差を計算
            int16_t error = target_rpm - current_rpm;

            if (!reached_target || abs(error) >= 50) {
                // PID制御を適用

                // 積分項の計算（積分誤差を更新）
                integral_error += error * dt_sec;
                // アンチワインドアップ処理
                constexpr float integral_limit = 500.0f;
                if (integral_error > integral_limit) integral_error = integral_limit;
                if (integral_error < -integral_limit) integral_error = -integral_limit;

                // 微分項の計算（フィルタリング）
                float derivative_error = (error - previous_error) / dt_sec;
                derivative_error_filtered = 
                    alpha * derivative_error + (1.0f - alpha) * derivative_error_filtered;
                previous_error = error; // 誤差を更新

                // 電流値を計算（PID制御）
                int16_t output_current = static_cast<int16_t>(
                    kp * error + ki * integral_error + kd * derivative_error_filtered);

                // 電流値の制限（範囲: -10000～10000）
                constexpr int16_t current_limit = 10000;
                if (output_current > current_limit) output_current = current_limit;
                if (output_current < -current_limit) output_current = -current_limit;

                // モーターに電流を設定
                motor.set_raw_current(output_current);

                printf("%d, %d, %d\n",
                       current_rpm, target_rpm, output_current);

                // 目標RPMに到達した場合、デバッグ用メッセージを出力
                if (abs(error) < 50) { // 許容誤差内
                    printf("Motor %d reached target RPM.\n", motor_id);
                    reached_target = true;
                }

            } else {
                // 目標RPMに到達後もPID制御を維持
                int16_t output_current = static_cast<int16_t>(
                    kp * error + ki * integral_error + kd * derivative_error_filtered);

                // 電流値の制限（範囲: -10000～10000）
                constexpr int16_t current_limit = 10000;
                if (output_current > current_limit) output_current = current_limit;
                if (output_current < -current_limit) output_current = -current_limit;

                // モーターに電流を設定
                motor.set_raw_current(output_current);

                printf("Maintaining target RPM: %d, current RPM: %d, output: %d\n",
                       target_rpm, current_rpm, output_current);
            }

            // CANメッセージを送信
            auto msgs = c610.to_msgs();
            if (!can.write(msgs[0]) || !can.write(msgs[1])) {
                // エラー処理（必要なら追加）
            }
        }
    }
}
