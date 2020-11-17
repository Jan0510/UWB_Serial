#!/usr/bin/python
# -*- coding: utf-8 -*-
# @Author  :   {Jan__}
# @Time    :   2020/11/17 14:37


import sys
from serial import Serial
from serial.tools import list_ports
from ui_demo_1 import Ui_Form
import numpy as np
from PyQt5.QtCore import QTimer
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from PyQt5.QtWidgets import QApplication, QMessageBox, QWidget


class Pyqt5_Serial(QWidget, Ui_Form):#QtWidgets.QWidget是QT库中的类。Ui_Form是QTDesigner生成的类
    #QWidget是QtWidgets模块下面的一个类，QWidget类是所有用户界面对象的基类
    #Pyqt5_Serial继承了2个父类
    def __init__(self):
        super(Pyqt5_Serial, self).__init__()#python2的写法？分别调用了2个父类的构造函数
        ##首先找到子类（Pyqt5_Serial）的父类（QWidget），然后把Pyqt5_Serial的对象self转成QWidget的对象，然后被转化的self调用自己的init函数
        try:
            self.setupUi(self)                  #setupUi方法需要2个参数，该句传入的参数为self，self
            self.init()                         #调用初始化函数
            self.ser = Serial()          #构造一个Serial类对象
            self.port_check()
            self.AP_Position = np.ones((3, 9))  # 基站群坐标列表,先创建一个全1矩阵
            self.AP_Position_Change_slot()      # 读取界面上AP坐标的默认参数
            self.numinWaiting = 0   # 缓冲区没有内容
            self.txt_write_Flag = 0   # 自动保存采样数据标志位
        except Exception as ex:
            print(ex)

    def init(self):
        self.modbus_reg = [0 for i in range(100)]  # 0-69是可读写数据，70-100是只读数据
        #低通滤波队列号的初始化
        self.index_in_LP_Array = 0
        self.LP_Array1 = [0 for i in range(20)]
        self.LP_Array2 = [0 for i in range(20)]
        self.LP_Array3 = [0 for i in range(20)]
        self.LP_Array4 = [0 for i in range(20)]
        self.LP_Array5 = [0 for i in range(20)]
        self.LP_Array6 = [0 for i in range(20)]
        self.LP_Array7 = [0 for i in range(20)]
        self.LP_Array8 = [0 for i in range(20)]
        self.LP_Array9 = [0 for i in range(20)]
        # 定时发送数据
        self.timer_send = QTimer()#实例化
        self.timer_send.timeout.connect(self.data_send)#绑定实例对象的timeout函数与data_send函数
        self.timer_send_cb.stateChanged.connect(self.data_send_timer)
        # 定时器接收数据
        self.rx_timer = QTimer(self)
        self.rx_timer.timeout.connect(self.data_receive)
        # 定时检测报文收发情况
        self.modbus_timer = QTimer(self)
        self.modbus_timer.timeout.connect(self.modbus_management_slot)
        # 03报文重发定时器
        self.modbus_03resend_timer = QTimer(self)
        self.modbus_03resend_timer.timeout.connect(self.modbus_03resend_slot)
        # 10报文重发定时器
        self.modbus_10resend_timer = QTimer(self)
        self.modbus_10resend_timer.timeout.connect(self.modbus_10resend_slot)
        # 接收数据和发送数据数目置零
        self.data_num_received = 0
        self.lineEdit.setText(str(self.data_num_received))#行文本显示
        self.data_num_sended = 0
        self.lineEdit_2.setText(str(self.data_num_sended))
        # 按钮信号连接
        self.btn_combine()
        # 按钮初始化。设备未连接时，多个按钮禁能
        self.disenable_btn()
    # 按钮信号与槽函数连接
    def btn_combine(self):
        # 天线校准页面的read按钮
        self.bnt_read_ANT_CAL.clicked.connect(self.bnt_read_ANT_CAL_slot)
        # 天线校准页面的write按钮
        self.bnt_write_ANT_CAL.clicked.connect(self.bnt_write_ANT_CAL_slot)
        # 连接设备按钮
        self.btn_connect_device.clicked.connect(self.device_connect_slot)
        # 基站/标签模式选择
        self.comboBox_Device_Mode.currentIndexChanged.connect(self.Device_Mode_change_slot)
        # 自动校准按钮
        self.btn_auto_calibration.clicked.connect(self.auto_calibration_slot)
        # 设定参数按钮
        self.btn_write_para.clicked.connect(self.btn_write_para_slot)
        # 读取参数按钮
        self.btn_read_para.clicked.connect(self.btn_read_para_slot)
        # txt开始记录按钮
        self.txt_Start.clicked.connect(self.txtStart_slot)
        # txt保存按钮
        self.txt_Save.clicked.connect(self.txtSave_slot)
        # AP的坐标，textChanged信号量都关联到同一个槽函数
        self.Spin_Box_x1.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_x2.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_x3.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_x4.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_x5.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_x6.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_x7.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_x8.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_x9.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_y1.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_y2.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_y3.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_y4.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_y5.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_y6.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_y7.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_y8.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_y9.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_z1.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_z2.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_z3.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_z4.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_z5.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_z6.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_z7.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_z8.valueChanged.connect(self.AP_Position_Change_slot)
        self.Spin_Box_z9.valueChanged.connect(self.AP_Position_Change_slot)
        # 清除发送窗口
        self.s3__clear_button.clicked.connect(self.send_data_clear)
        # 清除接收窗口
        self.s2__clear_button.clicked.connect(self.receive_data_clear)
        # 图像显示开始按钮
        self.Trace_Start.clicked.connect(self.Trace_Start_slot)
        # 图像显示停止按钮
        self.Trace_Stop.clicked.connect(self.Trace_Stop_slot)
        # 串口检测按钮
        self.s1__box_1.clicked.connect(self.port_check)  # 按钮clicked事件，连接函数port_check
        # 打开串口按钮
        self.open_button.clicked.connect(self.port_open)
        # 关闭串口按钮
        self.close_button.clicked.connect(self.port_close)
        # 发送数据按钮
        self.s3__send_button.clicked.connect(self.data_send)
    # 天线校准页面的read按钮
    def bnt_read_ANT_CAL_slot(self):
        # 显示天线延迟参数
        self.Spin_Box_DLY_1.setValue(self.modbus_reg[31])
        self.Spin_Box_DLY_2.setValue(self.modbus_reg[32])
        self.Spin_Box_DLY_3.setValue(self.modbus_reg[33])
        self.Spin_Box_DLY_4.setValue(self.modbus_reg[34])
        self.Spin_Box_DLY_5.setValue(self.modbus_reg[35])
        self.Spin_Box_DLY_6.setValue(self.modbus_reg[36])
        self.Spin_Box_DLY_7.setValue(self.modbus_reg[37])
        self.Spin_Box_DLY_8.setValue(self.modbus_reg[38])
        self.Spin_Box_DLY_9.setValue(self.modbus_reg[39])
        # 显示参考距离
        self.Spin_Box_DISTANCE_1.setValue(self.modbus_reg[11])
        self.Spin_Box_DISTANCE_2.setValue(self.modbus_reg[12])
        self.Spin_Box_DISTANCE_3.setValue(self.modbus_reg[13])
        self.Spin_Box_DISTANCE_4.setValue(self.modbus_reg[14])
        self.Spin_Box_DISTANCE_5.setValue(self.modbus_reg[15])
        self.Spin_Box_DISTANCE_6.setValue(self.modbus_reg[16])
        self.Spin_Box_DISTANCE_7.setValue(self.modbus_reg[17])
        self.Spin_Box_DISTANCE_8.setValue(self.modbus_reg[18])
        self.Spin_Box_DISTANCE_9.setValue(self.modbus_reg[19])
        # 显示参考温度
        self.Spin_Box_TEMPERATURE_1.setValue((self.modbus_reg[21] & 0xff00)/0x0100)
        self.Spin_Box_TEMPERATURE_2.setValue((self.modbus_reg[22] & 0xff00)/0x0100)
        self.Spin_Box_TEMPERATURE_3.setValue((self.modbus_reg[23] & 0xff00)/0x0100)
        self.Spin_Box_TEMPERATURE_4.setValue((self.modbus_reg[24] & 0xff00)/0x0100)
        self.Spin_Box_TEMPERATURE_5.setValue((self.modbus_reg[25] & 0xff00)/0x0100)
        self.Spin_Box_TEMPERATURE_6.setValue((self.modbus_reg[26] & 0xff00)/0x0100)
        self.Spin_Box_TEMPERATURE_7.setValue((self.modbus_reg[27] & 0xff00)/0x0100)
        self.Spin_Box_TEMPERATURE_8.setValue((self.modbus_reg[28] & 0xff00)/0x0100)
        self.Spin_Box_TEMPERATURE_9.setValue((self.modbus_reg[29] & 0xff00)/0x0100)
        # 显示参考电压
        self.Spin_Box_TEMPERATURE_1.setValue(self.modbus_reg[21] & 0x00ff)
        self.Spin_Box_TEMPERATURE_2.setValue(self.modbus_reg[22] & 0x00ff)
        self.Spin_Box_TEMPERATURE_3.setValue(self.modbus_reg[23] & 0x00ff)
        self.Spin_Box_TEMPERATURE_4.setValue(self.modbus_reg[24] & 0x00ff)
        self.Spin_Box_TEMPERATURE_5.setValue(self.modbus_reg[25] & 0x00ff)
        self.Spin_Box_TEMPERATURE_6.setValue(self.modbus_reg[26] & 0x00ff)
        self.Spin_Box_TEMPERATURE_7.setValue(self.modbus_reg[27] & 0x00ff)
        self.Spin_Box_TEMPERATURE_8.setValue(self.modbus_reg[28] & 0x00ff)
        self.Spin_Box_TEMPERATURE_9.setValue(self.modbus_reg[29] & 0x00ff)
       # 显示自动校准的激活状态
        self.Auto_Cal_Active_Box_1.setChecked(self.modbus_reg[41] & 0x0001)
        self.Auto_Cal_Active_Box_2.setChecked(self.modbus_reg[42] & 0x0001)
        self.Auto_Cal_Active_Box_3.setChecked(self.modbus_reg[43] & 0x0001)
        self.Auto_Cal_Active_Box_4.setChecked(self.modbus_reg[44] & 0x0001)
        self.Auto_Cal_Active_Box_5.setChecked(self.modbus_reg[45] & 0x0001)
        self.Auto_Cal_Active_Box_6.setChecked(self.modbus_reg[46] & 0x0001)
        self.Auto_Cal_Active_Box_7.setChecked(self.modbus_reg[47] & 0x0001)
        self.Auto_Cal_Active_Box_8.setChecked(self.modbus_reg[48] & 0x0001)
        self.Auto_Cal_Active_Box_9.setChecked(self.modbus_reg[49] & 0x0001)
    # 天线校准页面的write按钮
    def bnt_write_ANT_CAL_slot(self):
        for i in range(11,50):
            self.modbus_reg[i] = 0
        # 激活标志
        if self.Write_Flag__Box_1.isChecked():
            self.modbus_reg[11] = self.Spin_Box_DISTANCE_1.value()# 参考距离
            self.modbus_reg[21] = self.Spin_Box_TEMPERATURE_1.value() * 0x0100 +self.Spin_Box_VOLTAGE_1.value() # 参考温度# 参考电压
            self.modbus_reg[31] = self.Spin_Box_DLY_1.value()  # 天线延迟
            self.modbus_reg[41] = int(self.Auto_Cal_Active_Box_1.isChecked())
            self.modbus_reg[41] += 2
        if self.Write_Flag__Box_2.isChecked():
            self.modbus_reg[12] = self.Spin_Box_DISTANCE_2.value()# 参考距离
            self.modbus_reg[22] = self.Spin_Box_TEMPERATURE_2.value() * 0x0100 +self.Spin_Box_VOLTAGE_2.value() # 参考温度# 参考电压
            self.modbus_reg[32] = self.Spin_Box_DLY_2.value()  # 天线延迟
            self.modbus_reg[42] = int(self.Auto_Cal_Active_Box_2.isChecked())
            self.modbus_reg[42] += 2
        if self.Write_Flag__Box_3.isChecked():
            self.modbus_reg[13] = self.Spin_Box_DISTANCE_3.value()# 参考距离
            self.modbus_reg[23] = self.Spin_Box_TEMPERATURE_3.value() * 0x0100 +self.Spin_Box_VOLTAGE_3.value() # 参考温度# 参考电压
            self.modbus_reg[33] = self.Spin_Box_DLY_3.value()  # 天线延迟
            self.modbus_reg[43] = self.Auto_Cal_Active_Box_3.isChecked()
            self.modbus_reg[43] += 2
        if self.Write_Flag__Box_4.isChecked():
            self.modbus_reg[14] = self.Spin_Box_DISTANCE_4.value()# 参考距离
            self.modbus_reg[24] = self.Spin_Box_TEMPERATURE_4.value() * 0x0100 +self.Spin_Box_VOLTAGE_4.value() # 参考温度# 参考电压
            self.modbus_reg[34] = self.Spin_Box_DLY_4.value()  # 天线延迟
            self.modbus_reg[44] = int(self.Auto_Cal_Active_Box_4.isChecked())
            self.modbus_reg[44] += 2
        if self.Write_Flag__Box_5.isChecked():
            self.modbus_reg[15] = self.Spin_Box_DISTANCE_5.value()# 参考距离
            self.modbus_reg[25] = self.Spin_Box_TEMPERATURE_5.value() * 0x0100 +self.Spin_Box_VOLTAGE_5.value() # 参考温度# 参考电压
            self.modbus_reg[35] = self.Spin_Box_DLY_5.value()  # 天线延迟
            self.modbus_reg[45] = self.Auto_Cal_Active_Box_5.isChecked()
            self.modbus_reg[45] += 2
        if self.Write_Flag__Box_6.isChecked():
            self.modbus_reg[16] = self.Spin_Box_DISTANCE_6.value()# 参考距离
            self.modbus_reg[26] = self.Spin_Box_TEMPERATURE_6.value() * 0x0100 +self.Spin_Box_VOLTAGE_6.value() # 参考温度# 参考电压
            self.modbus_reg[36] = self.Spin_Box_DLY_6.value()  # 天线延迟
            self.modbus_reg[46] = int(self.Auto_Cal_Active_Box_6.isChecked())
            self.modbus_reg[46] += 2
        if self.Write_Flag__Box_7.isChecked():
            self.modbus_reg[17] = self.Spin_Box_DISTANCE_7.value()# 参考距离
            self.modbus_reg[27] = self.Spin_Box_TEMPERATURE_7.value() * 0x0100 +self.Spin_Box_VOLTAGE_7.value() # 参考温度# 参考电压
            self.modbus_reg[37] = self.Spin_Box_DLY_7.value()  # 天线延迟
            self.modbus_reg[47] = self.Auto_Cal_Active_Box_7.isChecked()
            self.modbus_reg[47] += 2
        if self.Write_Flag__Box_8.isChecked():
            self.modbus_reg[18] = self.Spin_Box_DISTANCE_8.value()# 参考距离
            self.modbus_reg[28] = self.Spin_Box_TEMPERATURE_8.value() * 0x0100 +self.Spin_Box_VOLTAGE_8.value() # 参考温度# 参考电压
            self.modbus_reg[38] = self.Spin_Box_DLY_8.value()  # 天线延迟
            self.modbus_reg[48] = int(self.Auto_Cal_Active_Box_8.isChecked())
            self.modbus_reg[48] += 2
        if self.Write_Flag__Box_9.isChecked():
            self.modbus_reg[19] = self.Spin_Box_DISTANCE_9.value()# 参考距离
            self.modbus_reg[29] = self.Spin_Box_TEMPERATURE_9.value() * 0x0100 +self.Spin_Box_VOLTAGE_9.value() # 参考温度# 参考电压
            self.modbus_reg[39] = self.Spin_Box_DLY_9.value()  # 天线延迟
            self.modbus_reg[49] = int(self.Auto_Cal_Active_Box_9.isChecked())
            self.modbus_reg[49] += 2
        self.if_btn_write_data_clicked = 1  # 设定数据按钮的点击标志位
        self.modbus_10cmd_wait = 1  # 1表示有报文需要发送，2表示有应答等待接收
        self.modbus_03cmd_wait = 0  # 暂停03
    #设备未连接时，多个按钮禁能
    def disenable_btn(self):
        self.if_device_connect = 0              # 连接标志位
        self.modbus_10cmd_wait = 0              # 10报文的发送标志位，若它=1，则03 06报文都要暂停发送，先发送10报文
        self.structure_Mode = 0                 # modbus通信的一个寄存器值，由于其不在窗口上显示，因此在这里初始化
        self.modbus_timer.stop()               # 定时发送03报文的定时器关闭
        self.btn_auto_calibration.setEnabled(0) #自动校准按钮
        self.btn_auto_calibration.setText("自动校准")
        self.btn_write_para.setEnabled(0)       #设置参数按钮
        self.btn_write_para.setText("设定参数")
        self.btn_read_para.setEnabled(0)        #读取参数按钮
        self.btn_read_para.setText("读取参数")
        self.Trace_Start.setEnabled(0)          #绘图开始按钮
        self.Trace_Stop.setEnabled(0)           #绘图结束按钮
    #设备连接成功，按钮使能
    def enable_btn(self):
        self.modbus_timer.start(100)            #定时检查
        self.btn_auto_calibration.setEnabled(1) #自动校准按钮
        self.btn_write_para.setEnabled(1)       #设置参数按钮
        self.if_btn_write_data_clicked = 0      #设定数据按钮的点击标志位
        self.btn_read_para.setEnabled(1)        #读取参数按钮
        self.if_btn_read_data_clicked = 0       #读取数据按钮的点击标志位
        self.Trace_Start.setEnabled(1)          #绘图开始按钮
        self.Trace_Stop.setEnabled(1)           #绘图结束按钮
    #连接设备，并读取参数
    def device_connect_slot(self):
        if self.ser.isOpen():#检测串口是否已打开
            self.if_device_connect = 0
            self.modbus_03cmd()
    #天线延迟自动校准
    def auto_calibration_slot(self):
        if self.structure_Mode == 16:
            self.structure_Mode = 0
            self.btn_auto_calibration.setText("自动校准")
        else:
            self.structure_Mode = 16     #标志位：定位的同时天线延迟自动校准
            self.btn_auto_calibration.setText("waiting...")
        # 0定位模式  16定位的同时自校准
        self.modbus_reg[4] = int(self.structure_Mode)
        # 天线延迟
        self.modbus_reg[5] = int(self.Spin_Box_Antdelay.value())
        # 天线延迟自校准的参考距离
        self.modbus_reg[6] = int(self.Spin_Box_reference_dis.value())
        self.modbus_10cmd_wait = 0  # 暂停10
        self.modbus_03cmd_wait = 0  # 暂停03
        self.modbus_10cmd(4, 3)      # 发送命令
    # 标签/基站模式选择
    def Device_Mode_change_slot(self):
        if 1 == self.comboBox_Device_Mode.currentIndex():
            self.spinBox_localAP_x.setEnabled(1)
            self.spinBox_localAP_y.setEnabled(1)
            self.spinBox_localAP_z.setEnabled(1)
        else:
            self.spinBox_localAP_x.setEnabled(0)
            self.spinBox_localAP_y.setEnabled(0)
            self.spinBox_localAP_z.setEnabled(0)
    #设置参数
    def btn_write_para_slot(self):
        self.if_btn_write_data_clicked = 1#设定数据按钮的点击标志位
        self.modbus_10cmd_wait = 1  # 1表示有报文需要发送，2表示有应答等待接收
        self.modbus_03cmd_wait = 0  # 暂停03
        # 设备串口通讯波特率0：4800,1：9600,2：14400,3：19200,4：38400,5：56000,6：57600,7：115200,8：128000,9：256000
        self.modbus_reg[0] = int(self.comboBox_serial_baudrate.currentIndex())
        # modbus地址
        self.modbus_reg[1] = int(self.spinBox_modbus_addr.value())
        # 基站编号
        if self.comboBox_device_ID.currentText() == 'Debug_AP':self.modbus_reg[2] = 100
        else:        self.modbus_reg[2] = int(self.comboBox_device_ID.currentIndex())
        # 标签/基站，模式
        self.modbus_reg[3] = int(self.comboBox_Device_Mode.currentIndex())
        # 0定位模式  16定位的同时自校准
        self.modbus_reg[4] = int(self.structure_Mode)
        # 天线延迟
        self.modbus_reg[5] = int(self.Spin_Box_Antdelay.value())
        #天线延迟自校准的参考距离
        self.modbus_reg[6] = int(self.Spin_Box_reference_dis.value())
        # 基站坐标y
        self.modbus_reg[8] = int(self.spinBox_localAP_y.value())
        # 基站坐标z
        self.modbus_reg[9] = int(self.spinBox_localAP_z.value())
        # 模块的空中码率，2=6.8Mbps,1=850Kbps,0=110Kbps
        self.modbus_reg[10] = int(self.comboBox_air_baudrate.currentIndex())
        # 由于python的int位数是变化的，因此下面几句话将int负数变成int16_t负数
        # 基站坐标x
        if int(self.spinBox_localAP_x.value()) < 0:
            self.modbus_reg[7] = 0x8000 + (32768-abs(self.spinBox_localAP_x.value()))
        else:
            # 非负数
            self.modbus_reg[7] = int(self.spinBox_localAP_x.value())
        # 基站坐标x
        if int(self.spinBox_localAP_y.value()) < 0:
            self.modbus_reg[8] = 0x8000 + (32768-abs(self.spinBox_localAP_y.value()))
        else:
            # 非负数
            self.modbus_reg[8] = int(self.spinBox_localAP_y.value())
        # 基站坐标z
        if int(self.spinBox_localAP_z.value()) < 0:
            self.modbus_reg[9] = 0x8000 + (32768-abs(self.spinBox_localAP_z.value()))
        else:
            # 非负数
            self.modbus_reg[9] = int(self.spinBox_localAP_z.value())
    #读取参数
    def btn_read_para_slot(self):
        # 如果是读取参数按钮被按下，则把modbus_reg队列的内容显示在窗口控件上
        # 这些参数一直是更新着的，只是没有显示在窗口控件上，目的是分离读、写功能
        # 更新窗口控件的数据
        # 设备串口波特率
        self.comboBox_serial_baudrate.setCurrentText(str(self.modbus_reg[0]))
        # modbus设备地址
        self.spinBox_modbus_addr.setValue(self.modbus_reg[1])
        # Device_ID 基站编号
        self.comboBox_device_ID.setCurrentIndex(self.modbus_reg[2])
        # Device_Mode 基站/标签
        self.comboBox_Device_Mode.setCurrentIndex(self.modbus_reg[3])
        # structure_Mode 定位/天线延迟自校准
        self.structure_Mode = self.modbus_reg[4]
        if self.structure_Mode == 16:
            self.btn_auto_calibration.setText("waiting...")
        else:
            self.btn_auto_calibration.setText("自动校准")
        # RX_DLY    天线延迟
        self.Spin_Box_Antdelay.setValue(self.modbus_reg[5])
        # ANT_Calibra_Distance  自校准的参考距离
        self.Spin_Box_reference_dis.setValue(self.modbus_reg[6])
        # LocalAP_x  基站坐标
        if self.modbus_reg[7] > 32767:  #若超过32767说明它在int16_t格式下是负数
            self.spinBox_localAP_x.setValue(self.modbus_reg[7]-32768-32768)
        else:
            self.spinBox_localAP_x.setValue(self.modbus_reg[7])
        # LocalAP_y  基站坐标
        if self.modbus_reg[8] > 32767:  # 若超过32767说明它在int16_t格式下是负数
            self.spinBox_localAP_y.setValue(self.modbus_reg[8]-32768-32768)
        else:
            self.spinBox_localAP_y.setValue(self.modbus_reg[8])
        # LocalAP_z  基站坐标
        if self.modbus_reg[9] > 32767:  # 若超过32767说明它在int16_t格式下是负数
            self.spinBox_localAP_z.setValue(self.modbus_reg[9]-32768-32768)
        else:
            self.spinBox_localAP_z.setValue(self.modbus_reg[9])
        # 模块的空中码率,2=6.8Mbps,1=850Kbps,0=110Kbps
        self.comboBox_air_baudrate.setCurrentIndex(self.modbus_reg[10])
        #天线校准页面的监控数据
        self.bnt_read_ANT_CAL_slot()
    #txt开始记录，槽函数
    def txtStart_slot(self):
        strName = self.lineEdit_txtName.text()
        if strName == '':           #如果没有设置文件名
            self.txt_Save.setEnabled(0)
            return 0
        self.txt_Start.setEnabled(0)
        self.txt_Save.setEnabled(1)
        self.txtFile = open("%s.txt"%strName, "a")# 设置文件对象，放在脚本文件夹下

        self.txtFile.seek(0)        #把文件光标定位到position 0
        self.txtFile.truncate()     #清空内容
        self.txt_write_Flag = 1     #实现连续写入的函数标志位=1
        #'a'表示可连续写入到文件，保留原内容，在原内容之后写入。


        #txt保存，槽函数
    def txtSave_slot(self):
        self.txtFile.close()        #关闭txt
        self.txt_write_Flag = 0     #实现连续写入的函数标志位=0
        self.txt_Start.setEnabled(1) #按钮使能

    def TimetoRedraw_slot(self):
        self.ax1.figure.canvas.draw()#重画当前图形,少了这句，坐标轴不会跟着更新。这用于更新已更改但不会自动重绘的图形。如果交互式模式为（ion()），则很少使用

    #坐标数据读取
    def AP_Position_Change_slot(self):
        self.AP_Position[0, 0] = self.Spin_Box_x1.value()
        self.AP_Position[0, 1] = self.Spin_Box_x2.value()
        self.AP_Position[0, 2] = self.Spin_Box_x3.value()
        self.AP_Position[0, 3] = self.Spin_Box_x4.value()
        self.AP_Position[0, 4] = self.Spin_Box_x5.value()
        self.AP_Position[0, 5] = self.Spin_Box_x6.value()
        self.AP_Position[1, 0] = self.Spin_Box_y1.value()
        self.AP_Position[1, 1] = self.Spin_Box_y2.value()
        self.AP_Position[1, 2] = self.Spin_Box_y3.value()
        self.AP_Position[1, 3] = self.Spin_Box_y4.value()
        self.AP_Position[1, 4] = self.Spin_Box_y5.value()
        self.AP_Position[1, 5] = self.Spin_Box_y6.value()
        self.AP_Position[2, 0] = self.Spin_Box_z1.value()
        self.AP_Position[2, 1] = self.Spin_Box_z2.value()
        self.AP_Position[2, 2] = self.Spin_Box_z3.value()
        self.AP_Position[2, 3] = self.Spin_Box_z4.value()
        self.AP_Position[2, 4] = self.Spin_Box_z5.value()
        self.AP_Position[2, 5] = self.Spin_Box_z6.value()
        self.AP_Position[0, 6] = self.Spin_Box_x7.value()
        self.AP_Position[1, 6] = self.Spin_Box_y7.value()
        self.AP_Position[2, 6] = self.Spin_Box_z7.value()
        self.AP_Position[0, 7] = self.Spin_Box_x8.value()
        self.AP_Position[1, 7] = self.Spin_Box_y8.value()
        self.AP_Position[2, 7] = self.Spin_Box_z8.value()
        self.AP_Position[0, 8] = self.Spin_Box_x9.value()
        self.AP_Position[1, 8] = self.Spin_Box_y9.value()
        self.AP_Position[2, 8] = self.Spin_Box_z9.value()
    # 动画函数的初始化函数
    def Trace_plot_Init(self):
        self.ax1.set(title='An Example Axes',ylabel='Y-Axis', xlabel='X-Axis')
        self.ax1.set_xlim(-100, 100)
        self.ax1.set_ylim(-100, 100)
        self.ax1.text(self.AP_Position[0, 0] + 2, self.AP_Position[1, 0] + 2, "AP_1", fontdict={'color': 'red'})
        self.ax1.text(self.AP_Position[0, 1] + 2, self.AP_Position[1, 1] + 2, "AP_2", fontdict={'color': 'red'})
        self.ax1.text(self.AP_Position[0, 2] + 2, self.AP_Position[1, 2] + 2, "AP_3", fontdict={'color': 'red'})
        self.ax1.text(self.AP_Position[0, 3] + 2, self.AP_Position[1, 3] + 2, "AP_4", fontdict={'color': 'red'})
        self.ax1.text(self.AP_Position[0, 4] + 2, self.AP_Position[1, 4] + 2, "AP_5", fontdict={'color': 'red'})
        self.ax1.text(self.AP_Position[0, 5] + 2, self.AP_Position[1, 5] + 2, "AP_6", fontdict={'color': 'red'})
        self.ax1.text(self.AP_Position[0, 6] + 2, self.AP_Position[1, 6] + 2, "AP_7", fontdict={'color': 'red'})

        line1, = self.ax1.plot(self.AP_Position[0, :], self.AP_Position[1, :], 'ro', label='基站群')
        line2, = self.ax1.plot(0, 0, 'bo', label='标签群')
        self.ax1.grid(True, linestyle='-.')

        #print(type(ln,))      #ln,加了逗号与没加时类型不一样

        return line1,line2,
    # 动画函数的更新函数
    def Trace_plot_Animation_update(self, frames):

        xmin, xmax = self.ax1.get_xlim()
        # 标签与原点的距离
        tag_distance = (self.Spin_Box_Tatrget_x.value()**2 + self.Spin_Box_Tatrget_y.value()**2)**0.5
        if xmax - tag_distance > 120:
            #缩小边界
            xmax = tag_distance+75
            self.ax1.set_xlim(-xmax, xmax)
            self.ax1.set_ylim(-xmax, xmax)
            self.ax1.figure.canvas.draw()  # 重画当前图形,少了这句，坐标轴不会跟着更新。
        elif xmax - tag_distance < 50:
            #扩大边界
            xmax = tag_distance+75
            self.ax1.set_xlim(-xmax, xmax)
            self.ax1.set_ylim(-xmax, xmax)
            self.ax1.figure.canvas.draw()  # 重画当前图形,少了这句，坐标轴不会跟着更新。
        # 标签处于当前平面范围，则考虑是否需要缩小范围
        line1, line2, = self.ax1.plot(self.AP_Position[0, :], self.AP_Position[1, :], 'ro', self.Spin_Box_Tatrget_x.value(), self.Spin_Box_Tatrget_y.value(), 'bo')
        return line1, line2
    #显示图像
    def Trace_Start_slot(self):
        self.Trace_Start.setEnabled(0)
        plt.rcParams['font.sans-serif'] = ['SimHei']    # 用来正常显示中文标签
        plt.rcParams['axes.unicode_minus'] = False      # 用来正常显示负号
        self.fig = plt.figure(1)
        self.fig.suptitle('The Target Position Trace 标签定位', fontsize=14, fontweight='bold')
        self.ax1 = self.fig.add_subplot()  #生成子图，相当于fig = plt.figure(),ax = fig.add_subplot(),其中ax的函数参数表示把当前画布进行分割，例：fig.add_subplot(2,2,2).表示将画布分割为两行两列,ax在第2个子图中绘制，其中行优先，
        self.ani = FuncAnimation(self.fig, self.Trace_plot_Animation_update, interval=10,
                                 init_func=self.Trace_plot_Init, blit=True)
        # plt.ion()  # 打开交互模式
        # 简而言之，就是如果不是显式声明，即通过 ani = FuncAnimation() 赋值，Python的垃圾回收机制就会将这个动画引用删除。所以最简单的方式就是通过ani = FuncAnimation() 的方式来保存动画。
        self.index_in_LP_Array = 0          #低通滤波队列索引，运行到这里时，该实例变量才被创建
        plt.show()
    # 关闭图像
    def Trace_Stop_slot(self):
        self.Trace_Start.setEnabled(1)
        plt.close()
    # 串口检测
    def port_check(self):
        # 检测所有存在的串口，将信息存储在字典中
        self.Com_Dict = {}
        port_list = list(list_ports.comports())#获取串口列表
        #调用list把一个字典转换成一个列表，port_list是个2维数组。
        self.s1__box_2.clear()
        for port in port_list:
            #如果直接for 循环一个list 的时候，那么每次循环的值都是这个list 里面的元素，因此1个port包含2个数据
            self.Com_Dict["%s" % port[0]] = "%s" % port[1]#把port[1]内的字符串传给port[0]
            #"%s:%d"%("ab",3) => "ab:3"，%s表示格化式一个对象为字符，%d表示整数。
            #port[0]是串口名，port[1]是串口号
            self.s1__box_2.addItem(port[0])#增加
        if len(self.Com_Dict) == 0:
            self.open_button.setEnabled(0)
            return 0
        else:
            self.open_button.setEnabled(1)

            return 1
    # 打开串口
    def port_open(self):
        if '' == self.s1__box_2.currentText():
            print("检测不到串口")
            self.open_button.setEnabled(1)
            return None
        self.ser.port = self.s1__box_2.currentText()#串口号
        self.ser.baudrate = int(self.s1__box_3.currentText())#波特率
        self.ser.bytesize = int(self.s1__box_4.currentText())#数据位
        self.ser.stopbits = int(self.s1__box_6.currentText())#停止位
        self.ser.parity = self.s1__box_5.currentText()#校验位
        #try...except..处理异常的结构
        try:#被监控的程序段
            self.ser.open()
        except:#若发送异常则执行的程序段
            QMessageBox.critical(self, "Port Error", "此串口不能被打开！")
            print("串口不能打开")
            return None

        # 打开串口接收定时器，周期为10ms
        self.rx_timer.start(10)
        if self.ser.isOpen():
            self.open_button.setEnabled(False)#禁能
            self.close_button.setEnabled(True)#使能
    # 关闭串口
    def port_close(self):
        self.rx_timer.stop()
        self.timer_send.stop()
        self.disenable_btn()    #关闭串口，断开设备连接，按钮禁能
        plt.close()
        try:
            self.ser.close()
        except:
            pass
        self.open_button.setEnabled(True)
        self.close_button.setEnabled(False)
        self.lineEdit_3.setEnabled(True)
        # 接收数据和发送数据数目置零
        self.data_num_received = 0
        self.lineEdit.setText(str(self.data_num_received))
        self.data_num_sended = 0
        self.lineEdit_2.setText(str(self.data_num_sended))
    # 发送数据
    def data_send(self):
        if self.ser.isOpen():
            input_s = self.s3__send_text.toPlainText()#先获取文本，判断是否空白
            if input_s != "":
                # 非空字符串
                if self.hex_send.isChecked():#返回ture是选中该按钮
                    # hex发送
                    input_s = input_s.strip() #去除首尾空白符(包括\n、\r、\t、' ')
                    send_list = []#列表
                    while input_s != '':#非空，执行到字符串空了为止
                        try:#取2个字节转成16位的数
                            num = int(input_s[0:2], 16)#str[a：b]是字符串的截取操作，第a位到b-1位，若a=-1，表示倒数第1位
                        except ValueError:
                            QMessageBox.critical(self, 'wrong data', '请输入十六进制数据，以空格分开!')
                            return None
                        input_s = input_s[2:].strip()#去除首尾空白符(包括\n、\r、\t、' ')
                        send_list.append(num)   #添加一个新行
                    input_s = bytes(send_list)  #列表转成字节序列
                else:
                    # ascii发送
                    #input_s = (input_s + '\r\n').encode('utf-8')
                    input_s = (input_s).encode('utf-8')
                num = self.ser.write(input_s)#串口写并返回字节数
                self.data_num_sended += num
                self.lineEdit_2.setText(str(self.data_num_sended))
        else:
            pass
    #应答数据解析
    def modbus_Receive_Translate(self,data,length):
        # 在__init__函数外定义，则创建实例对象时该self.modbus_reg不会被创建
        # 只当执行到modbus_Receive_Translate函数时才会创建，并作为实例变量一直存在
        data_crc = getcrc16(data, length-2)
        if data[0] == 0x01 and data_crc//256 == data[length-2] and data_crc % 256 == data[length-1]:
            if data[1] == 0x03:#03 功能码，读
                if self.modbus_03cmd_wait == 0:
                    # 如果标志位=0，该功能被暂停，有其他优先级更高的报文在传输。
                    # 目的是不希望modbus_reg中的内容被其他报文改写
                    return None
                reg_num = data[2]    #字节数目
                for i in range(0, reg_num//2):#把数据填入modbus_reg队列
                    self.modbus_reg[i] = data[3+i*2]*256 + data[4+i*2]
                self.updata_read_only_data()       # 更新窗口控件的显示数据
                self.modbus_03cmd_wait = 0  # 等待应答标志清除
                # 如果启动了保存采样数据的功能
                if self.txt_write_Flag > 0:
                    self.txtFile.writelines(str(self.modbus_reg[70:79:] + self.modbus_reg[90:93:]))  # 写入列表
                    self.txtFile.write('\n')
                    self.txt_write_Flag += 1
                    if self.txt_write_Flag > self.spinBox_AutoSaveNum.value():
                        self.txtSave_slot()
                        self.txt_write_Flag = 0
                # 连接成功标志位,只在关闭串口时写0
                if self.if_device_connect == 0:
                    self.if_device_connect = 1
                    QMessageBox.information(None, "提示", "已成功连接设备！", QMessageBox.Yes)
                    self.enable_btn()
                    self.btn_read_para_slot()#更新窗口数据
                # 如果是读取数据按钮  点击时发送的03报文 则收到应答的同时更新窗口数据
                # 即以下数据的更新，只发送在第一次连接设备时、点击读取数据按钮时
            elif data[1] == 0x06:
                # 写命令的应答报文只检查CRC就算了
                self.modbus_03cmd_wait = 0
            elif data[1] == 0x10:
                # 写命令的应答
                self.modbus_10resend_timer.stop()  # 关闭定时器
                self.modbus_10cmd_wait = 0
                self.modbus_03cmd_wait = 0
                if self.if_btn_write_data_clicked == 1:
                    self.if_btn_write_data_clicked = 0
                    QMessageBox.information(None, "提示", "参数写入成功！", QMessageBox.Yes)
                elif self.structure_Mode == 16 and "waiting..." == self.btn_auto_calibration.text():
                    # 上位机发送自动校准命令成功，得到应答
                    QMessageBox.information(None, "提示", "天线延迟校准需要一段时间，再次点击按钮可停止校准！", QMessageBox.Yes)
                elif self.structure_Mode == 0 and "自动校准" == self.btn_auto_calibration.text():
                    # 上位机发送取消校准命令成功，得到应答
                    QMessageBox.information(None, "提示", "自动校准已取消！", QMessageBox.Yes)
        else:
            print("CRC_ERROR")
    #窗口控件更新数据
    def updata_read_only_data(self):
        # 对采样距离进行均值滤波
        # self.LP_Array1[self.index_in_LP_Array] = self.modbus_reg[30]
        # self. [self.index_in_LP_Array] = self.modbus_reg[31]
        # self.LP_Array3[self.index_in_LP_Array] = self.modbus_reg[32]
        # self.LP_Array4[self.index_in_LP_Array] = self.modbus_reg[33]
        # self.LP_Array5[self.index_in_LP_Array] = self.modbus_reg[34]
        # self.LP_Array6[self.index_in_LP_Array] = self.modbus_reg[35]
        # self.LP_Array7[self.index_in_LP_Array] = self.modbus_reg[36]
        # self.LP_Array8[self.index_in_LP_Array] = self.modbus_reg[37]
        # self.LP_Array9[self.index_in_LP_Array] = self.modbus_reg[38]
        # LP_Result = [np.mean(self.LP_Array1),
        #              np.mean(self.LP_Array2),
        #              np.mean(self.LP_Array3),
        #              np.mean(self.LP_Array4),
        #              np.mean(self.LP_Array5),
        #              np.mean(self.LP_Array6),
        #              np.mean(self.LP_Array7),
        #              np.mean(self.LP_Array8),
        #              np.mean(self.LP_Array9)]
        # self.index_in_LP_Array += 1
        # if self.index_in_LP_Array == 20:
        #     self.index_in_LP_Array = 0
        #采样距离
        self.Spin_Box_Dis1.setValue(self.modbus_reg[70])
        self.Spin_Box_Dis2.setValue(self.modbus_reg[71])
        self.Spin_Box_Dis3.setValue(self.modbus_reg[72])
        self.Spin_Box_Dis4.setValue(self.modbus_reg[73])
        self.Spin_Box_Dis5.setValue(self.modbus_reg[74])
        self.Spin_Box_Dis6.setValue(self.modbus_reg[75])
        self.Spin_Box_Dis7.setValue(self.modbus_reg[76])
        self.Spin_Box_Dis8.setValue(self.modbus_reg[77])
        self.Spin_Box_Dis9.setValue(self.modbus_reg[78])

        for i in range(9):
            #采样距离
            item = self.tableWidget_monitor.item(i, 0)
            item.setText(str(self.modbus_reg[70 + i]))
            #采样温度
            item = self.tableWidget_monitor.item(i, 3)
            item.setText("%.4f" % ((self.modbus_reg[80+i]&0xff00)//256 * 1.13 - 113.0))

            #采样电压
            item = self.tableWidget_monitor.item(i, 4)
            # item.setText(str((self.modbus_reg[80+i] & 0x00ff)*0.0057+2.3))
            item.setText("%.4f" % ((self.modbus_reg[80 + i] & 0x00ff) * 0.0057 + 2.3))

            #参考距离
            item = self.tableWidget_monitor.item(i, 1)
            item.setText(str(self.modbus_reg[11 + i]))

            #天线延迟
            item = self.tableWidget_monitor.item(i, 2)
            item.setText(str(self.modbus_reg[31 + i]))

        # 标签坐标,若超过32767说明它在int16_t格式下是负数
        if self.modbus_reg[90] > 32767:
            self.Spin_Box_Tatrget_x.setValue(self.modbus_reg[90]-32768-32768)
        else:
            self.Spin_Box_Tatrget_x.setValue(self.modbus_reg[90])
        if self.modbus_reg[91] > 32767:
            self.Spin_Box_Tatrget_y.setValue(self.modbus_reg[91]-32768-32768)
        else:
            self.Spin_Box_Tatrget_y.setValue(self.modbus_reg[91])
        if self.modbus_reg[92] > 32767:
            self.Spin_Box_Tatrget_z.setValue(self.modbus_reg[92]-32768-32768)
        else:
            self.Spin_Box_Tatrget_z.setValue(self.modbus_reg[92])

        #一个特殊的寄存器,=16表示定位的同时校准天线延迟，=0表示定位，但不校准天线延迟
        self.structure_Mode = self.modbus_reg[4]
        if self.structure_Mode == 0 and "waiting..." == self.btn_auto_calibration.text():
            self.btn_auto_calibration.setText("自动校准")
            QMessageBox.information(None, "提示", " 自动校准已完成！", QMessageBox.Yes)
            # RX_DLY    天线延迟
            self.Spin_Box_Antdelay.setValue(self.modbus_reg[5])
            # ANT_Calibra_Distance  自校准的参考距离
            self.Spin_Box_reference_dis.setValue(self.modbus_reg[6])
        # #
        # elif self.structure_Mode == 16 and "自动校准" == self.btn_auto_calibration.text():
        #     self.btn_auto_calibration.setText("自动校准")
    #读数据
    def modbus_03cmd(self):
        send_buf = bytearray(8)#返回一个长度为 8 的初始化数组
        send_buf[0] = 0x01    #地址
        send_buf[1] = 0x03    #功能码
        send_buf[2] = 0   # startaddr
        send_buf[3] = 0   # 从0开始，读取modbus_reg寄存器列表
        send_buf[4] = 0   # number
        send_buf[5] = 100  # 100个寄存器
        crc16 = getcrc16(send_buf, 6)
        send_buf[6] = crc16 // 256
        send_buf[7] = crc16 % 256
        self.modbus_03cmd_wait = 2          # 等待应答标志位
        num = self.ser.write(send_buf)      # 串口写并返回字节数
        self.data_num_sended += num
        self.lineEdit_2.setText(str(self.data_num_sended))
    #写单个数据，似乎没用上06的
    def modbus_06cmd(self,startaddr,data):
        if self.ser.isOpen():
            if self.modbus_03cmd_wait==0:
                send_buf = bytearray(10)#返回一个长度为 10 的初始化数组
                send_buf[0] = 0x01    #地址
                send_buf[1] = 0x06    #功能码
                send_buf[2] = startaddr/256   # startaddr
                send_buf[3] = startaddr%256   # startaddr
                send_buf[4] = 0  # number
                send_buf[5] = 1  # number
                send_buf[6] = data / 256
                send_buf[7] = data % 256
                crc16 = getcrc16(send_buf, 8)
                send_buf[8] = crc16 / 256
                send_buf[9] = crc16 % 256
                self.modbus_06cmd_wait = 1              #等待应答标志位
                num = self.ser.write(send_buf, 10)  # 串口写并返回字节数
                self.data_num_sended += num
                self.lineEdit_2.setText(str(self.data_num_sended))
    #写多个数据
    def modbus_10cmd(self,start_addr,length):
        send_buf = bytearray(8+length*2)  # 返回一个长度为 * 的初始化数组
        send_buf[0] = 0x01  # 地址
        send_buf[1] = 0x10  # 功能码
        send_buf[2] = start_addr//256   # startaddr
        send_buf[3] = start_addr%256   # startaddr
        send_buf[4] = length//256  # number
        send_buf[5] = length%256  # number

        for i in range(length):
            send_buf[6+i*2] = int(self.modbus_reg[start_addr+i]) // 256
            send_buf[7+i*2] = int(self.modbus_reg[start_addr+i]) % 256

        # 目前只用到modbus_reg0-9 后续有必要再增加

        crc16 = getcrc16(send_buf, 8+length*2-2)

        send_buf[8+length*2-2] = crc16 // 256
        send_buf[8+length*2-1] = crc16 % 256
        self.modbus_10cmd_wait = 2      # 等待应答标志位
        num = self.ser.write(send_buf)  # 串口写并返回字节数
        self.data_num_sended += num
        self.lineEdit_2.setText(str(self.data_num_sended))
    # modbus 收发数据的 超时、重发、优先级管理函数
    def modbus_management_slot(self):
        # 是否已经连接成功
        if self.if_device_connect == 1:
            # 1表示等待发送，2表示等待应答，0表示无
            if self.modbus_03cmd_wait == 2 or self.modbus_10cmd_wait == 2:
            # 若串口目前需要等待应答报文，则等就好了
            # 重发定时器有时候莫名其妙关闭了，在这里打个补丁，检测是否异常关闭
                if self.modbus_10cmd_wait == 2 and self.modbus_10resend_timer.isActive() == False:
                    self.modbus_10cmd_wait = 1
                return None
            # 10等待发送
            elif self.modbus_10cmd_wait == 1:
                self.modbus_10resend_timer.start(100)  # 定时器开
                self.modbus_10cmd(0, 100)
                return None
            # 03等待发送
            elif self.modbus_03cmd_wait == 1:
                self.modbus_03cmd()
                return None
            #  如果没有任何标志，则发送查询
            else:
                self.modbus_03cmd_wait = 1
    # 应答超时槽函数
    def modbus_03resend_slot(self):
        self.modbus_03resend_timer.stop()       #关闭定时器
        self.modbus_03cmd_wait = 1              #打开标志位，重新发送03指令
    def modbus_10resend_slot(self):
        self.modbus_10resend_timer.stop()
        self.modbus_10cmd_wait = 1              # 打开标志位，重新发送10指令
        self.modbus_03cmd_wait = 0              # 暂停03
    # 串口接收
    def data_receive(self):
        try:
            num = self.ser.inWaiting()#返回接收缓存中的字节数
        except:
            self.port_close()
            return None
        if num > 0:
            if self.numinWaiting == num:    # 缓冲区数量不变，说明数据帧接收完成
                data = self.ser.read(num)   # 读取num个数据
                self.modbus_Receive_Translate(data, num)#modbus数据解析
                # 将串口数据传给edit text部件，hex显示
                if self.hex_receive.checkState():  # 16进制显示按钮被选中时
                    out_s = ''
                    for i in range(0, len(data)):
                        out_s = out_s + '{:02X}'.format(data[i]) + ' '  # format()中的内容填入大括号里，：02X表示左补0，2个位对齐，16进制
                    self.s2__receive_text.insertPlainText(out_s)  #
                else:  # 以字符串显示
                    # 串口接收到的字符串为b'123',要转化成unicode字符串才能输出到窗口中去
                    self.s2__receive_text.insertPlainText(data.decode('iso-8859-1'))
                # 统计接收字符的数量
                self.data_num_received += num
                self.lineEdit.setText(str(self.data_num_received))
                self.numinWaiting = 0       # 缓冲区数量清零
            else:
                self.numinWaiting = num     # 缓冲区数量还在增加，更新数值后返回
        else:
            self.numinWaiting = 0           # 缓冲区没有内容
        return 0
    # 定时发送数据
    def data_send_timer(self):
        if self.timer_send_cb.isChecked():#检查定时发送选框
            self.timer_send.start(int(self.lineEdit_3.text()))
            self.lineEdit_3.setEnabled(False)
        else:
            self.timer_send.stop()
            self.lineEdit_3.setEnabled(True)

    # 清除显示
    def send_data_clear(self):
        self.s3__send_text.setText("")
    # 清除接收窗口
    def receive_data_clear(self):
        self.s2__receive_text.setText("")


aucCRCLo = [
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC,
    0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8,
    0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14,
    0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0,
    0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C,
    0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28,
    0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4,
    0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0,
    0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C,
    0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78,
    0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4,
    0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50,
    0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C,
    0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44,
    0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40,
]

aucCRCHi = [
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
]

def getcrc16(frame, length): #传入要校验的数组名及其长度
    CRCHi = 0xff
    CRCLo = 0xff
    for i in range(0, length):
        i_index = CRCHi ^ (frame[i])
        CRCHi = (CRCLo ^ aucCRCHi[i_index])
        CRCLo = aucCRCLo[i_index]
    return CRCHi*256 + CRCLo    #CRC校验返回值 // CRCHI向左移动，就是逆序计算的代表

"""if __name__ == '__main__'的意思是：
当.py文件被直接运行时，if __name__ == '__main__'之下的代码块将被运行；
当.py文件以模块形式被导入时，if __name__ == '__main__'之下的代码块不被运行。"""
if __name__ == '__main__':

    app = QApplication(sys.argv)# 实例化一个应用对象，sys.argv是一组命令行参数的列表。Python可以在shell里运行，这是一种通过参数来选择启动脚本的方式。
    myshow = Pyqt5_Serial()
    myshow.show() # 让控件在桌面上显示出来。控件在内存里创建，之后才能在显示器上显示出来。
    sys.exit(app.exec_())# 确保主循环安全退出

