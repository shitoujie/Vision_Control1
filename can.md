
nuc上can0底盘，can1云台

gimbal(can0):
201 ammor
202 ammol

205 yaw
206 pitch

203 拨盘


chassis(can1):
    //can202-----battary-----can201
    //|                         |
    //|                         |
    //|                         |
    //|                         |
    //|                         |
    //|                         |
    //|                         |
    //can203-----------------can204

6020(pitch, yaw):
标识符：0x204+ 驱动器 ID
01机械角度， 23转速， 45实际转矩电流， 6电机温度， 7NULL

驱动 0x1FF:01 id1, 23 id2, 45 id3, 67 id4


3508(wheels, ammol, ammor):
标识符：0x200 + 电调 ID
01机械角度， 23转速， 45实际转矩电流， 6电机温度， 7NULL

驱动 0x200:01 id1, 23 id2, 45 id3, 67 id4



2006(拨盘):
标识符：0x200 + 电调 ID
01机械角度， 23转速， 45实际转矩， 67NULL

驱动 0x200:01 id1, 23 id2, 45 id3, 67 id4



