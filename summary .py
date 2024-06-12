from hub import light_matrix, motion_sensor, port, sound
import motor_pair, motor, color_sensor, force_sensor
import runloop, time, math


## 省略語一覧
# dist : distance,距離
# cw : clockwise,時計回りの、正回転の
# col : color,色
# curr : current,現在の
# mcn : machine,マシン
# tgt : target,目標の
# velo : velocity,速度
# circum : circumference,円周
# conv : convert,変換する
# mot : motor,モーター
# ang : angle,角度
# orienttn : orientation,向き
# addtnl : additional,追加の、足される
# val : value,値
# prev : previous,前の
# max : maximum,最大の
# min : minimum,最少の
# w : with,と
# gyr : gyro sensor,ジャイロセンサー
# til : until,まで
# elmnt : element,エレメント、要素
# diff : differential,微分、差異
# estmtn : estimation,推移
# jdge : judge,判定、
# ave : average,平均
# posi : positive,正の
# nega : negative,負の


### グローバル変数 ###

## マシンスペック定数
TIRE_DIAMETER = 86#mm# タイヤの直径
TIRE_CIRCUM = math.pi * TIRE_DIAMETER
DIST_BETWEEN_TIRES = 162#mm# タイヤ間の距離

motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)
DRIVE_MOTORS = motor_pair.PAIR_1# 駆動モーターのポート
LEFT_DRIVE_MOTOR = port.B# 左の駆動モーター
RIGHT_DRIVE_MOTOR = port.A# 右の駆動モーター
CW_DRIVE_MOTOR = port.B# 正回転する測定用の駆動モーターのポート
DOWNWARD_COL_SENSOR = port.C# 下向きのカラーセンサーのポート
SIDEWAYS_COL_SENSOR = port.D# 横向きのカラーセンサーのポート

## 定数
LEFT_DIRECTION = 1
RIGHT_DIRECTION = -1

TILT_YAW = 0
RGB_RED = 0
RGB_GREEN = 1
RGB_BLUE = 2
HSB_HUE = 0
HSB_SATURATION = 1
HSB_BRIGHTNESS = 2


## マシン状態変数
CURR_MCN_ORIENTATION = 0# マシンの現在の向き




### モジュール関数 ###
## 戻り値なし
# マシンの現在の向きを更新する
def update_mcn_orienttn(rotate_direction, rotation_angle):# (回転方向, 回転角度)
    global CURR_MCN_ORIENTATION

    addtnl_deci_degrees = rotate_direction * rotation_angle *10# 加算される角度(デシ度)

    if CURR_MCN_ORIENTATION + addtnl_deci_degrees >= 1800 or CURR_MCN_ORIENTATION + addtnl_deci_degrees < -1800:# +と-の境を超えたとき
        CURR_MCN_ORIENTATION = rotate_direction * -3600 + CURR_MCN_ORIENTATION

    CURR_MCN_ORIENTATION += addtnl_deci_degrees



## 戻り値あり
# 数値の符号を得る
def sign_of_value(value):# (対象の値)
    return abs(value)/value if value != 0 else 0# 1かｰ1か0を返す


# 距離をモーターの回転角度に変換
def conv_dist_to_mot_ang(distance):# (変換前の距離)
    global TIRE_CIRCUM
    return distance/TIRE_CIRCUM*360# 変換


# RGB値をHSB値に変換
def conv_rgb_to_hsb(rgb_value):# (変換前の値)
    global RGB_RED, RGB_GREEN, RGB_BLUE

    # 値の設定
    red_value = rgb_value[RGB_RED]
    green_value = rgb_value[RGB_GREEN]
    blue_value = rgb_value[RGB_BLUE]
    hue_value = 0
    saturation_value = 0
    brightness_value = 0
    max_value = max(red_value, green_value, blue_value)
    min_value = min(red_value, green_value, blue_value)
    if max_value == min_value:max_value += 1# 差が0になるのを防ぐためにインクリメントする

    # 色相の導出
    if max_value == red_value:
        hue_value = 60 * ((green_value-blue_value) / (max_value-min_value))
    elif max_value == green_value:
        hue_value = 60 * ((blue_value*red_value) / (max_value-min_value)) +120
    elif max_value == blue_value:
        hue_value = 60 * ((red_value*green_value) / (max_value-min_value)) +240
    if hue_value < 0:hue_value += 360
    # 彩度の導出
    saturation_value = (max_value-min_value) / max_value
    # 輝度の導出
    brightness_value = max_value

    return (hue_value, saturation_value, brightness_value)


# pid制御値を求める
def pid_control_value(kp, ki, kd, error, prev_error):# (kp値, ki値, kd値, 誤差, 前回の誤差)
    dt = 1# dt*実行回数=1
    proportional_val = error# 比例制御値
    integral_val = (error + prev_error)/2 *dt# 積分制御値
    differential_val = (error - prev_error) / dt# 微分制御値
    return kp*proportional_val + ki*integral_val + kd*differential_val# pid制御値




### 足回りの関数 ###

## 基礎的な関数
# 前進後退
async def move(tgt_distance, tgt_velocity, stop_mode):# (目標距離, 目標角速度, 停止方式)
    global DRIVE_MOTORS

    tgt_motor_angle = int(conv_dist_to_mot_ang(tgt_distance))# モーターの目標回転角度

    motor_pair.move_for_degrees(DRIVE_MOTORS, tgt_motor_angle, 0, velocity= tgt_velocity, stop= stop_mode)


# 左スピン
async def spin_to_left(tgt_angle, tgt_velocity):# (目標角度, 目標速度)
    global DIST_BETWEEN_TIRES, DRIVE_MOTORS

    tgt_distance = math.pi*DIST_BETWEEN_TIRES * tgt_angle/360# 目標距離
    tgt_motor_angle = (conv_dist_to_mot_ang(tgt_distance))# モーター―の目標回転角度

    motor_pair.move_for_degrees(DRIVE_MOTORS, tgt_motor_angle, 100, velocity= tgt_velocity)
    update_mcn_orienttn(LEFT_DIRECTION, tgt_angle)# CURR_MCN_ORIENTATIONの更新


# 右スピン
async def spin_to_right(tgt_angle, tgt_velocity):# (目標角度, 目標速度)
    global DIST_BETWEEN_TIRES, DRIVE_MOTORS

    tgt_distance = math.pi*DIST_BETWEEN_TIRES * tgt_angle/360# 目標距離
    tgt_motor_angle = (conv_dist_to_mot_ang(tgt_distance))# モーター―の目標回転角度

    motor_pair.move_for_degrees(DRIVE_MOTORS, tgt_motor_angle, -100, velocity= tgt_velocity)
    update_mcn_orienttn(RIGHT_DIRECTION, tgt_angle)# CURR_MCN_ORIENTATIONの更新



## センサーを利用する関数
# 黒を検知するまで前進後退
async def move_until_black(tgt_velocity, stop_mode, additional_distance):# (目標速度, 停止方式, 追加距離)
    global DOWNWARD_COL_SENSOR, HSB_BRIGHTNESS, DRIVE_MOTORS

    curr_rgb_val = []# 現在のRGB値
    curr_brightness_val = 0# 現在の輝度
    threshold_value = 100# 黒になる閾値

    while curr_brightness_val > threshold_value:# 輝度の値が閾値より多い間, 黒線まで
        curr_rgb_val = color_sensor.rgbi(DOWNWARD_COL_SENSOR)# 現在のRGB値
        curr_brightness_val = conv_rgb_to_hsb(curr_rgb_val)[HSB_BRIGHTNESS]# 現在の輝度
        motor_pair.move(DRIVE_MOTORS, 0, velocity= tgt_velocity)

    if additional_distance != 0:# 追加で進む距離があるなら
        tgt_motor_angle = conv_dist_to_mot_ang(additional_distance)# 目標のモーターの回転角度
        motor_pair.move_for_degrees(DRIVE_MOTORS, tgt_motor_angle, 0, velocity= tgt_velocity)

    motor_pair.stop(DRIVE_MOTORS, stop= stop_mode)


# ジャイロセンサーを使った前進後退
async def move_using_gyro(tgt_distance, tgt_velocity, stop_mode):# (目標距離, 目標速度, 停止方式)
    global CW_DRIVE_MOTOR, TILT_YAW, CURR_MCN_ORIENTATION, DRIVE_MOTORS

    tgt_motor_angle = int(conv_dist_to_mot_ang(tgt_distance))# 目標のモーターの回転角度
    curr_motor_angle = 0# 現在のモーターの回転角度

    kp = 1
    ki = 1
    kd = 1
    error = 0
    prev_error = 0

    motor.reset_relative_position(CW_DRIVE_MOTOR, 0)

    while curr_motor_angle < tgt_motor_angle:# 目標のモーターの回転角度になるまで

        curr_mcn_yaw_angle = motion_sensor.tilt_angles()[TILT_YAW]# 現在のヨー角
        error = curr_mcn_yaw_angle - CURR_MCN_ORIENTATION# 誤差
        control_value = int(pid_control_value(kp, ki, kd, error, prev_error))# 制御値

        motor_pair.move_tank(DRIVE_MOTORS, tgt_velocity+control_value, tgt_velocity-control_value)

        prev_error = error# 前回の誤差
        curr_motor_angle = motor.relative_position(CW_DRIVE_MOTOR)

    motor_pair.stop(DRIVE_MOTORS, stop= stop_mode)


# 黒を検知するまで、ジャイロセンサーを使って前進後退
async def move_w_gyr_til_black(tgt_velocity, stop_mode, additional_distance):# (目標速度,停止方式,追加距離)
    global DOWNWARD_COL_SENSOR, HSB_BRIGHTNESS, TILT_YAW, CURR_MCN_ORIENTATION, DRIVE_MOTORS

    curr_rgb_val = []# 現在のRGB値
    curr_brightness_val = 0# 現在の輝度
    threshold_value = 100# 黒になる閾値

    kp = 1
    ki = 1
    kd = 1
    error = 0
    prev_error = 0

    while curr_brightness_val > threshold_value:# 黒まで
        curr_rgb_val = color_sensor.rgbi(DOWNWARD_COL_SENSOR)# 現在のRGB値
        curr_brightness_val = conv_rgb_to_hsb(curr_rgb_val)[HSB_BRIGHTNESS]# 現在の輝度

        curr_mcn_yaw_angle = motion_sensor.tilt_angles()[TILT_YAW]# 現在のヨー角
        error = curr_mcn_yaw_angle - CURR_MCN_ORIENTATION# 誤差
        control_value = int(pid_control_value(kp, ki, kd, error, prev_error))# 制御値

        motor_pair.move_tank(DRIVE_MOTORS, tgt_velocity+control_value, tgt_velocity-control_value)

        prev_error = error# 前回の誤差

    if additional_distance != 0:# 追加で進む距離があるなら
        tgt_motor_angle = conv_dist_to_mot_ang(additional_distance)# 現在のモーターの回転角度

        motor.reset_relative_position(CW_DRIVE_MOTOR, 0)
        curr_motor_angle = 0
        prev_error = 0
        while curr_motor_angle < tgt_motor_angle:# 目標のモーターの回転角度まで

            curr_mcn_yaw_angle = motion_sensor.tilt_angles()[TILT_YAW]# 現在のヨー角
            error = curr_mcn_yaw_angle - CURR_MCN_ORIENTATION# 誤差
            control_value = int(pid_control_value(kp, ki, kd, error, prev_error))# 制御値

            motor_pair.move_tank(DRIVE_MOTORS, tgt_velocity+control_value, tgt_velocity-control_value)

            prev_error = error# 前回の誤差
            curr_motor_angle = motor.relative_position(CW_DRIVE_MOTOR)#現在のモーターの回転角度

    motor_pair.stop(DRIVE_MOTORS, stop= stop_mode)



## 応用的な関数
# ターンの一般化された関数
async def turn(outer_motor, turning_radius, tgt_angle, tgt_velocity, stop_mode):# (回転の外側のモーター, 回転半径, 目標角度, 目標速度, 停止方式)
    global LEFT_DRIVE_MOTOR, RIGHT_DRIVE_MOTOR, DIST_BETWEEN_TIRES

    # モーターの設定
    outer_port = outer_motor
    inner_port = LEFT_DRIVE_MOTOR if outer_motor == RIGHT_DRIVE_MOTOR else RIGHT_DRIVE_MOTOR
    # モーターの回転角度の設定
    outer_tgt_mot_angle = conv_dist_to_mot_ang(math.pi*2*turning_radius *tgt_angle/360)
    inner_tgt_mot_angle = conv_dist_to_mot_ang(math.pi*2*abs(DIST_BETWEEN_TIRES-turning_radius) *tgt_angle/360)
    # 目標速度の設定
    outer_tgt_velocity = tgt_velocity
    inner_tgt_velocity = tgt_velocity*(DIST_BETWEEN_TIRES-turning_radius)/turning_radius

    motor.run_for_degrees(outer_port, outer_tgt_mot_angle, outer_tgt_velocity, stop= stop_mode)
    await motor.run_for_degrees(inner_port, inner_tgt_mot_angle, inner_tgt_velocity, stop= stop_mode)

    # 現在の向きの更新
    update_mcn_orienttn(sign_of_value(outer_tgt_mot_angle), tgt_angle)



### 工程の関数 ###
## 色の検知判定関数
# RGB値の取得
async def get_rgb_values():
    global CW_DRIVE_MOTOR, TILT_YAW, CURR_MCN_ORIENTATION, DRIVE_MOTORS

    width_of_element = 32#mm# エレメントの幅
    dist_between_elmnts = 52#mm# エレメント間の距離
    width_of_a_section = width_of_element + dist_between_elmnts# 一区間の幅
    number_of_elements = 6# エレメントの個数
    width_of_sections = number_of_elements * width_of_a_section# 全区間の幅
    additional_distance = 150#mm# 追加で進む距離

    tgt_distance = width_of_sections + additional_distance# 目標距離
    tgt_motor_angle = conv_dist_to_mot_ang(tgt_distance)# 目標のモーターの回転角度
    tgt_velocity = 650# 目標速度
    curr_motor_angle = 0# 現在のモーターの回転角度

    kp = 1
    ki = 1
    kd = 1
    error = 0
    prev_error = 0
    rgb_data = []

    motor.reset_relative_position(CW_DRIVE_MOTOR, 0)
    while curr_motor_angle < tgt_motor_angle:# 目標のモーターの回転角度になるまで

        curr_mcn_yaw_angle = motion_sensor.tilt_angles()[TILT_YAW]# 現在のヨー角
        error = curr_mcn_yaw_angle - CURR_MCN_ORIENTATION# 誤差
        control_value = int(pid_control_value(kp, ki, kd, error, prev_error))# 制御値

        motor_pair.move_tank(DRIVE_MOTORS, tgt_velocity+control_value, tgt_velocity-control_value)
        rgb_data.append(color_sensor.rgbi(SIDEWAYS_COL_SENSOR))# RGB値の取得

        prev_error = error# 限界の誤差
        curr_motor_angle = motor.relative_position(CW_DRIVE_MOTOR)# 現在のモーターの回転角度

    motor_pair.stop(DRIVE_MOTORS, stop= motor.BRAKE)

    return rgb_data


# 黒か青かの判定
async def judge_blue_or_balck(rgb_raw_data):# (RGB値)
    global RGB_GREEN, HSB_BRIGHTNESS

    rgb_data = []# RGB値
    processed_rgb_data = [[],[],[]]# 処理されたRGB値
    wave_points = [[],[]]# 波の位置
    judgement_result = []# 判定の結果
    number_of_elements = 6# エレメントの個数


    # データの設定
    for data in rgb_raw_data:# タプルをリストに変換
        for index in range(3):
            rgb_data[index].append(data[index])
    for i in range(len(rgb_data[0])):# RGBデータを3つごとの平均データにする
        if i%3 == 0:
            for j in range(len(rgb_data)):
                processed_rgb_data[j].append((rgb_data[j][i]+rgb_data[j][i-1]+rgb_data[j][i-2])/3)

    green_data = processed_rgb_data[RGB_GREEN]# 緑データ
    hsb_data = conv_rgb_to_hsb(processed_rgb_data)# HSBデータ
    brightness_data = hsb_data[HSB_BRIGHTNESS]# 輝度データ
    ave_of_green_data = sum(green_data)/len(green_data)# Gの平均データ
    diff_data_for_estmtn = [green_data[index] - green_data[index-1] for index in range(1, len(green_data))]# 推定のための微分データ
    diff_data_for_estmtn.insert(0,0)
    intgrl_data_for_jdge = [0,0,0,0,0,0]# 判定のための積分データ
    ave_of_integral_data = [0,0,0,0,0,0]# 積分データの各波での平均

    # 波の位置推定
    is_within_wave = False# 波の範囲の中にいるか
    sum_of_posi_val = sum(filter(lambda x: x>0, diff_data_for_estmtn))# 微分データの正値の合計
    sum_of_nega_val = sum(filter(lambda x: x<0, diff_data_for_estmtn))# 微分データの負値の合計
    upper_threshold = sum_of_posi_val / len(diff_data_for_estmtn)# 波の始点を示す閾値
    lower_threshold = sum_of_nega_val / len(diff_data_for_estmtn)# 波の終点を示す閾値

    for index, datum in enumerate(diff_data_for_estmtn):
        if is_within_wave == False and datum > upper_threshold and green_data[index] > ave_of_green_data:# 波の範囲外で閾値を超えており、緑データが十分に大きいなら
            wave_points[0].append(index-1)
            is_within_wave = True
        elif is_within_wave == True and datum < lower_threshold and diff_data_for_estmtn[index+1] > lower_threshold:# 波の範囲内で閾値を超えており、次の微分データが閾値を超えていないなら
            wave_points[1].append(index+1)
            is_within_wave = False
        if len(wave_points[0]) >= number_of_elements and len(wave_points[1]) >= number_of_elements:# 始点と終点の数がエレメントの個数になったら
            break

    # 色の判定
    for i in range(len(wave_points[0])):
        intgrl_data_for_jdge[i] = sum(brightness_data[wave_points[0][i] : wave_points[1][i]+1])# 判定のための積分データ
        ave_of_integral_data[i] = intgrl_data_for_jdge[i] / (wave_points[1][i]+1 - wave_points[0][1])# 積分データの各波での平均
    threshold_for_jdge = sum(ave_of_integral_data) / len(ave_of_integral_data)# 判定の閾値

    for datum in ave_of_integral_data:
        judgement_result.append("blue" if datum > threshold_for_jdge else "black")# 判定結果

    return judgement_result



### メイン関数 ###
async def main():
    global RIGHT_DRIVE_MOTOR, DIST_BETWEEN_TIRES

    await move(300, 650, motor.BRAKE)
    await move(-300, 650, motor.BRAKE)
    await spin_to_left(90, 500)
    await spin_to_right(90, 500)
    await spin_to_left(150, 500)
    await spin_to_right(30, 500)
    await move_until_black(500, motor.BRAKE, -50)
    await move_until_black(-500, motor.BRAKE, 50)
    await move_using_gyro(200, 500, motor.BRAKE)
    await move_using_gyro(200, -500, motor.BRAKE)
    await move_w_gyr_til_black(500, motor.BRAKE, -10)
    await move_w_gyr_til_black(-500, motor.BRAKE, 10)
    await turn(RIGHT_DRIVE_MOTOR, DIST_BETWEEN_TIRES/2, 90, 500, motor.BRAKE)
    await turn(LEFT_DRIVE_MOTOR, DIST_BETWEEN_TIRES/2, 90, -500, motor.BRAKE)
    await turn(RIGHT_DRIVE_MOTOR, DIST_BETWEEN_TIRES/1.5, 90, 500, motor.BRAKE)
    await turn(LEFT_DRIVE_MOTOR, DIST_BETWEEN_TIRES/1.5, 90, -500, motor.BRAKE)
    await turn(LEFT_DRIVE_MOTOR, DIST_BETWEEN_TIRES/1.5, 90, 500, motor.BRAKE)
    await turn(RIGHT_DRIVE_MOTOR, DIST_BETWEEN_TIRES/1.5, 90, -500, motor.BRAKE)
    await turn(RIGHT_DRIVE_MOTOR, DIST_BETWEEN_TIRES, 90, 500, motor.BRAKE)
    await turn(LEFT_DRIVE_MOTOR, DIST_BETWEEN_TIRES, 90, -500, motor.BRAKE)
    await turn(RIGHT_DRIVE_MOTOR, DIST_BETWEEN_TIRES, -90, 500, motor.BRAKE)
    await turn(LEFT_DRIVE_MOTOR, DIST_BETWEEN_TIRES, 90, 500, motor.BRAKE)
    await turn(RIGHT_DRIVE_MOTOR, DIST_BETWEEN_TIRES*1.2, 90, 500, motor.BRAKE)
    await turn(LEFT_DRIVE_MOTOR, DIST_BETWEEN_TIRES*1.2, 90, -500, motor.BRAKE)
    await turn(LEFT_DRIVE_MOTOR, DIST_BETWEEN_TIRES*1.2, 90, 500, motor.BRAKE)
    await turn(RIGHT_DRIVE_MOTOR, DIST_BETWEEN_TIRES*1.2, 90, -500, motor.BRAKE)

    rgb_data = await get_rgb_values()
    judgement_result = await judge_blue_or_balck(rgb_data)
    print(judgement_result)


### 実行 ###
start_time = time.ticks_ms() / 1000

runloop.run(main())

end_time = time.ticks_ms() / 1000
elapsed_time = end_time - start_time
print("time:" , elapsed_time)