#清扫车控制指令

#mcu部分
uint8 brake	        #电磁刹指令 0开;1关
uint8 gear	        #挡位 0空挡;1后退;2前进;3保留
uint16 rpm	        #转速 量程：0-6000，对应实际电机转速0-6000rpm

#eps部分
float32 angle	    #轮端转向角度 分辨率0.2° [-66.0,66.0] 适当缩减
uint16 angle_speed	#转向角速度 120-1500rpm

#vcu部分
bool sweep          #一键清扫 true:清扫 false:不清扫 (优先级低于各独立开关)
int32 sweep_mode    #清扫模式: 0=标准模式(刷子), 1=混合模式(刷子+洒水)

# 各清扫部件独立控制 (当sweep为true时，以下开关可以单独控制对应部件)
bool enable_main_brush      # 主刷电机
bool enable_vacuum          # 吸尘电机
bool enable_dust_shake      # 振尘电机
bool enable_main_brush_pole # 主刷推杆 (下沉/抬升)
bool enable_flap_pole       # 前挡皮推杆电机
bool enable_side_brush      # 边刷电机
bool enable_water_pump      # 水泵电机
