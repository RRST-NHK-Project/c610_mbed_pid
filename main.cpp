#include <mbed.h>
#include "C610.hpp"

CAN can{PA_11, PA_12, (int)1e6};
C610Array c610{};

// PID制御の比例ゲイン（P制御のみを使用）
constexpr float kp = 3.50f; // 比例ゲイン
constexpr uint16_t rotation_angle = 180; // 回転させたい角度（単位: 度）
constexpr uint16_t max_angle = 8192;     // 1周分の角度

int main() {
  printf("setup\n");

  const int motor_id = 1; // 制御するモーターのID（1～8）
  uint16_t target_angle = 0; // 初期目標角度
  bool target_set = false;   // 目標角度が設定されたかどうか

  while (1) {
    auto now = HighResClock::now();
    static auto pre = now;
    auto dt = now - pre;

    // CANメッセージを読み取り
    if (CANMessage msg; can.read(msg)) {
      c610.parse_packet(msg);
    }

    // 10msごとに制御
    if (dt > 10ms) {
      pre = now;

      auto& motor = c610[motor_id - 1]; // 対応するモーターを取得
      uint16_t current_angle = motor.get_angle(); // 現在の角度を取得

      // 初回のみ目標角度を設定
      if (!target_set) {
        target_angle = (current_angle + rotation_angle * max_angle / 360) % max_angle;
        target_set = true;
        printf("Target angle set to: %d (Current angle: %d)\n", target_angle, current_angle);
      }

      // 角度の誤差を計算（循環補正を考慮）
      int16_t error = target_angle - current_angle;
      if (error > max_angle / 2) error -= max_angle; // 最短経路に補正
      if (error < -max_angle / 2) error += max_angle;

      // 電流値を計算（P制御）
      int16_t output_current = static_cast<int16_t>(kp * error);

      // 電流値の制限（範囲: -10000～10000）
      if (output_current > 10000) output_current = 10000;
      if (output_current < -10000) output_current = -10000;

      // モーターに電流を設定
      motor.set_raw_current(output_current);

      // デバッグ用出力
      printf("Motor %d: Target Angle: %d, Current Angle: %d, Output Current: %d\n",
             motor_id, target_angle, current_angle, output_current);

      // 目標角度に到達した場合、停止
      if (abs(error) < 5) { // 許容誤差内
        motor.set_raw_current(0); // モーター停止
        printf("Motor %d reached target angle.\n", motor_id);
        break; // ループを抜けるか、必要なら別の処理を追加
      }

      // CANメッセージを送信
      auto msgs = c610.to_msgs();
      if (!can.write(msgs[0]) || !can.write(msgs[1])) {
        printf("Failed to write c610 msg\n");
      }
    }
  }
}
//github