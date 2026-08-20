# 双腿轮腿离地检测与落地恢复方案

## 1. 状态与建模变量

主动状态流转：

~~~text
NORMAL -> AIR -> NORMAL
~~~

建模状态使用双腿平均实际腿长：

\[
L=(L_1+L_2)/2,\qquad
\dot L=(\dot L_1+\dot L_2)/2
\]

L 是平均物理腿长，不是旧模型中的平均竖直投影高度。L1、L2 仍用于支持力、落地压缩和单腿闭环。

已统一：

- H、DOT_H -> L、DOT_L
- dot_h -> dot_L
- AIRBORNE -> AIR
- landing_recovery_pending -> landing_recovery

当前 LQR 的 L1/L2 二维增益调度保留，不能在未重新生成系数前改成单变量调度。

## 2. 加速度坐标转换

使用 INS 的去重力机体加速度：

~~~cpp
ins->get_accel_without_g_b(&ax_b, &ay_b, &az_b);
~~~

机体坐标系到航向坐标系：

\[
R_B^Y=R_y(\theta)R_x(\phi)
\]

竖直加速度：

\[
a_z^Y =
-\sin\theta\,a_x^B
+\cos\theta\sin\phi\,a_y^B
+\cos\theta\cos\phi\,a_z^B
\]

a_z^Y 使用低通滤波，支持力结果再经过独立低通。腿长和摆角二阶导数由有限差分得到后进行低通和限幅。

## 3. 支持力计算

优先使用电机反馈反推的虚拟力：

~~~text
F_L = current_F_L
T_p = current_T_p
~~~

第 i 条腿的腿端竖直惯性项：

\[
\begin{aligned}
a_{P_i,z}={}&a_z^Y-\ddot L_i\cos\beta_i
+2\dot L_i\dot\beta_i\sin\beta_i\\
&+L_i\ddot\beta_i\sin\beta_i
+L_i\dot\beta_i^2\cos\beta_i
\end{aligned}
\]

支持力估计：

\[
P_i=
F_{L,i}\cos\beta_i
+\frac{T_{p,i}}{L_i}\sin\beta_i
+m_{\mathrm{eff},i}a_{P_i,z}
+b_i
\]

说明：

- F_L 是腿轴向力，不是竖直支持力；
- T_p/L_i 是摆动力矩换算出的切向力；
- 加速度项必须乘等效质量，单位才是 N；
- 去重力加速度已经通过 get_accel_without_g_b() 获得，不能再额外添加 m_eff*g；
- 静态时应满足 P_left > 0、P_right > 0，并且 P_left + P_right 接近机器人重量。

## 4. 离地检测

NORMAL 中默认使用双腿同时低支持力：

~~~text
P_left  < P_takeoff_on
P_right < P_takeoff_on
~~~

两个条件连续满足防抖时间后切入 AIR。使用双腿 AND 可避免单腿卸载、侧倾和过台阶造成误判。

阈值使用迟滞：

~~~text
进入 AIR:  P_left、P_right 都低于 P_takeoff_on
解除离地:  P_left + P_right 高于 P_contact_off
~~~

阈值和防抖时间必须通过静态站立、抬车、单腿卸载和跳跃日志标定。

## 5. AIR 控制

AIR 中不调用完整的正常 LQR 输出。LQR 只使用左右腿摆角：

\[
T_{p,i}^{AIR}
=-
\left[
K_{\beta,i}(\beta_i^{ref}-\beta_i)
+K_{\dot\beta,i}(0-\dot\beta_i)
\right]
\]

不使用 x、dot_x、俯仰、轮平衡、轮运动、转向和 LQR 伸缩通道。

腿长使用独立 PD，并将每条腿目标从进入 AIR 时的当前值限速伸展到 0.32 m 左右。

车轮使用速度阻尼锁定：

\[
T_{w,i}^{lock}=-K_{w,lock}\dot\vartheta_i
\]

不能用 send_torque(0) 代替锁轮。落地确认后清零锁轮力矩，恢复正常轮控制。

## 6. 落地与恢复

AIR 中要求冲击和腿长压缩同时出现：

~~~text
impact =
    accel_z_y_lpf > landing_acc_on

compressed =
    (L1 < L1_air_ref - delta_L_land) ||
    (L2 < L2_air_ref - delta_L_land)

landing_candidate = impact && compressed
~~~

落地确认后：

1. 切回 NORMAL；
2. 设置 landing_recovery=true；
3. 读取当前平均腿长：
   \[
   L_{land}=(L_1+L_2)/2
   \]
4. 将目标 L 立即设为 L_land；
5. 正常 LQR 立即恢复；
6. 目标 L 以限速轨迹恢复到 L_nominal=0.20 m；
7. 目标误差足够小且总支持力稳定后清除 landing_recovery。

state_normal_t::enter() 在 landing_recovery 期间不能无条件重置目标腿长。

## 7. 已实现的数据和函数

新增空中数据包括：

~~~cpp
chassis_state_t state;
bool landing_recovery;
uint16_t takeoff_counter;
uint16_t landing_counter;
float L_ref;
float L_air_ref[2];
float accel_z_y;
float accel_z_y_lpf;
float support_force[2];
float support_force_sum;
~~~

核心处理函数：

~~~cpp
_update_accel_heading_frame();
_calc_support_force();
_detect_takeoff();
_detect_landing();
_execute_air_control();
_execute_landing_recovery();
~~~

## 8. 验证项目

1. 静态站立时两腿支持力为正，总和接近机器人重量。
2. 单腿卸载不会立即进入 AIR。
3. 双腿抬起后支持力连续下降并进入 AIR。
4. Pitch、Roll 同时变化时竖直加速度投影符号正确。
5. AIR 中轮速趋近零，LQR 只有摆角通道产生摆动力矩。
6. 空中腿长平滑伸展到约 0.32 m。
7. 落地时加速度冲击和腿长压缩同时出现。
8. 落地后目标 L 先锁定当前平均腿长，再平滑恢复到 0.20 m。
9. 支持力计算保持 N 单位，未重复计算重力。
