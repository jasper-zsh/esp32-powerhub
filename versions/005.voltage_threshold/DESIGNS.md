# 设计文档 - v005 电压阈值读取扩展

## 目标
- 扩展 BLE 特征值 0xFFF5 的读取与通知载荷，在原有电压、温度及温度阈值基础上，新增睡眠电压阈值与唤醒电压阈值字段，形成对称的数据返回格式。
- 保持写命令集不变，客户端兼容性仅需调整载荷解析。

## GATT 数据格式
### READ 响应（总长 16 字节）
```
Byte0-1   : Vin_mV (uint16, BE)
Byte2-3   : Temp_centi (int16, BE)
Byte4-5   : Temp_high_centi (int16, BE)
Byte6-7   : Temp_recover_centi (int16, BE)
Byte8-9   : Sleep_threshold_mV (uint16, BE)
Byte10-11 : Wake_threshold_mV (uint16, BE)
Byte12    : Status_flags (bit0=热保护, bit1=温度有效, bit2-7=保留)
Byte13    : Reserved (0)
Byte14-15 : Reserved (0)
```

### NOTIFY 载荷（总长 12 字节）
```
Byte0-1 : Vin_mV (uint16, BE)
Byte2-3 : Temp_centi (int16, BE)
Byte4-5 : Temp_high_centi (int16, BE)
Byte6-7 : Temp_recover_centi (int16, BE)
Byte8-9 : Sleep_threshold_mV (uint16, BE)
Byte10-11 : Wake_threshold_mV (uint16, BE)
```

> 说明：保持字段顺序与 READ 响应前六个字段一致，便于客户端共享解析逻辑。

## 数据来源
- `power_mgr` 已维护 `s_sleep_mv`、`s_wake_mv` 变量并可通过现有 setter 被写入；无需新增存储接口。
- READ/NOTIFY 时直接取当前内存值填充；阈值更新后下一帧通知自动携带最新值。

## 兼容性
- 旧版客户端若仍按 12 字节 READ 解析会出现错位，因此需在版本发布说明中强调字节布局变更；本次改动仅在服务器侧，通过文档同步客户端升级。
- 写命令格式不变，状态标志位定义保持原样。

## 测试计划
1. 构建后通过 BLE 读取 0xFFF5，确认返回长度为 16 字节，字段数值正确。
2. 写入 0x01 与 0x02 命令修改电压阈值，再读取和订阅通知，确认数据更新。
3. 订阅通知，观察 1Hz 推送长度为 12 字节且包含阈值。
4. 回归检查温度阈值与状态标志输出仍正常。
