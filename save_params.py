# -*- coding: utf-8 -*-
p = r'C:\Users\qaz06\workspace_ccstheia\ESP32_WiFi_Bridge\调参记录_20260718_b100稳定.txt'
content = '''============================================================
灰度巡迹小车 - 调参记录
保存时间: 2026-07-18
状态: b100 跑得挺好 (用户原话, 初始参数 + f1000)
============================================================

【当前参数 (b100 稳定运行)】
  基础速度    base_speed    = 100
  转向P       turn_p        = 20    (默认)
  转向D       turn_d        = 15
  转弯速度    pivot_speed   = 100   (默认)
  前冲时间    corner_fwd_ms = 1000  (用户调整, 原默认500)

【命令序列 (恢复到这个状态)】
  m0
  b100
  f1000
  g

【硬件配置】
  MCU:     MSPM0G3507 (天猛星)
  WiFi:    ESP32S3 (DNESP32S3M最小系统板)
  灰度:    亚博 YB-MVX05 八路 (CD4051+LM393)
  驱动:    四路电机驱动板 (I2C控制)

【接线】
  灰度: AD0=PB4 AD1=PB5 AD2=PB2 OUT=PA15
  I2C:  SCL=PA16 SDA=PA1
  UART: PA8(TX)->ESP32 IO18(RX2)
        PA9(RX)<-ESP32 IO17(TX2)
  电机: M4=左轮 M2=右轮 (代码用负号修正转向)

【WiFi】
  SSID: KineticRobotics-5G
  PWD:  Kinkin#123123
  TCP:  8080

【直角弯逻辑】
  检测: CH0+CH1+CH2 都亮->左转 / CH5+CH6+CH7 都亮->右转
  前冲: 固定时间 (f命令调节, 默认500, 当前1000)
  转弯: 停车原地转, CH3或CH4亮就停
  超时: 无 (取消时间限制)

【灰度传感器布局】
  CH0  CH1  CH2  CH3  CH4  CH5  CH6  CH7
  -3.5 -2.5 -1.5 -0.5 +0.5 +1.5 +2.5 +3.5
  左                                     右

【下一步目标】
  速度提升一倍 (b100 -> b200)
============================================================
'''
with open(p, 'w', encoding='utf-8') as f:
    f.write(content)
print(f'OK: {len(content)} bytes')
