import sys
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox,QLabel
from PyQt5.QtCore import QObject, QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QPen, QColor, QBrush, QIcon,QPixmap
import re
from QCustomPlot_PyQt5 import QCustomPlot, QCPGraph, QCPScatterStyle
from GUI_MPU import Ui_MainWindow
import serial.tools.list_ports
from PyQt5.QtWidgets import QMessageBox
from functools import partial
import cv2
import requests
import os
import math
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap, QPen, QColor
from PyQt5.QtGui import QPainter
global mode, distance, address, data, t_plot, roll_value,pitch_value,Roll_u,Pitch_u,roll_ref,yaw_ref,pitch_ref, Kp,Kd,Ki,J1_FL ,J2_FL, J3_FL, J1_FR, J2_FR, J3_FR, J1_BL, J2_BL, J3_BL, J1_BR, J2_BR, J3_BR,a1,a2,a3,step
data="0";mode = ""; t_plot = []; a_plot = []; b_plot = [];c_plot =[]; Roll_u =0.00; roll_ref =0.00; pitch_ref = 0.00;yaw_ref=0.0;Pitch_u =0.00;roll_value = 000.00; pitch_value = 000.00; yaw_value = 000.00 ;Kp = 00.00;Kd =00.00;Ki=00.00
J1_FL=000.00 ;J2_FL=000.00; J3_FL=000.00; J1_FR=000.00; J2_FR=000.00; J3_FR=000.00; J1_BL=000.00; J2_BL=000.00; J3_BL=000.00; J1_BR=000.00; J2_BR=000.00; J3_BR=000.00; a1 = 56.84; a2 = 114.00 ; a3 = 170.74
class SerialReader(QObject):
    data_received = pyqtSignal(str)

    def __init__(self, serial_port):
        super().__init__()
        self.serial_port = serial_port
        self.running = False

    def run(self):
        self.running = True
        while self.running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    received_data = self.serial_port.readline().decode("utf-8").strip()
                    self.data_received.emit(received_data)
                except Exception as e:
                    print(f"Error reading data: {e}")
    def stop(self):
        self.running = False
URL = "http://192.168.43.73"

class MainWindow:
    def __init__(self):
        super().__init__()
        self.main_win = QMainWindow()
        self.uic = Ui_MainWindow()
        self.uic.setupUi(self.main_win)
        self.serial_port = None
        self.serial_reader = None
        self.init_serial()
        self.textbox_servo_map = {
            self.uic.J1_FL_SV: 0,
            self.uic.J2_FL_SV: 1,
            self.uic.J3_FL_SV: 2,
            self.uic.J1_FR_SV: 9,
            self.uic.J2_FR_SV: 10,
            self.uic.J3_FR_SV: 11,
            self.uic.J1_BL_SV: 3,
            self.uic.J2_BL_SV: 4,
            self.uic.J3_BL_SV: 5,
            self.uic.J1_BR_SV: 6,
            self.uic.J2_BR_SV: 7,
            self.uic.J3_BR_SV: 8
        }
        
        # Gán giá trị khởi tạo vào các trường văn bản
        self.uic.J1_FL.setText(str(137))
        self.uic.J2_FL.setText(str(75))
        self.uic.J3_FL.setText(str(-120))
        self.uic.J1_FR.setText(str(137))
        self.uic.J2_FR.setText(str(0))
        self.uic.J3_FR.setText(str(-120))
        self.uic.J1_BL.setText(str(137))
        self.uic.J2_BL.setText(str(-75))
        self.uic.J3_BL.setText(str(-120))
        self.uic.J1_BR.setText(str(137))
        self.uic.J2_BR.setText(str(0))
        self.uic.J3_BR.setText(str(-120))
       
        self.uic.J1_FL_SV.setText(str(90))
        self.uic.J2_FL_SV.setText(str(90))
        self.uic.J3_FL_SV.setText(str(90))
        self.uic.J1_FR_SV.setText(str(90))
        self.uic.J2_FR_SV.setText(str(90))
        self.uic.J3_FR_SV.setText(str(90))
        self.uic.J1_BL_SV.setText(str(90))
        self.uic.J2_BL_SV.setText(str(90))
        self.uic.J3_BL_SV.setText(str(90))
        self.uic.J1_BR_SV.setText(str(90))
        self.uic.J2_BR_SV.setText(str(90))
        self.uic.J3_BR_SV.setText(str(90))

        self.uic.Kp.setText(str(0.0))
        self.uic.Kd.setText(str(0.0))
        self.uic.Ki.setText(str(0.0))

        self.uic.roll_ref.setText(str(0))
        self.uic.pitch_ref.setText(str(0))
        self.uic.yaw_ref.setText(str(0))
        self.uic.step.setText(str(0))

        self.uic.distance.setText(str(0))
#option combo_servoID
        self.uic.combo_IDservo.addItem("-")
        self.uic.combo_IDservo.addItems(["FL_J1", "FL_J2", "FL_J3", "FR_J1","FR_J2","FR_J3","BL_J1","BL_J2","BL_J3","BR_J1","BR_J2","BR_J3"])
#option comboport
        self.uic.comboBox_port.addItem(QIcon("port_icon.png"), "-")
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.uic.comboBox_port.addItem(port.device)
        self.uic.comboBox_port.currentIndexChanged.connect(self.active_button)
#option combodata
        self.uic.comboBox_baudrate.addItem("-")
        self.uic.comboBox_baudrate.addItems(["9600", "19200", "38400", "57600","115200"])
        self.uic.comboBox_baudrate.currentIndexChanged.connect(self.active_button)        
        self.uic.pushButton_connect.setDisabled(True)
        self.uic.pushButton_connect.clicked.connect(self.connect_serial)
#Button Plot
        self.uic.plot_btn.clicked.connect(self.plot)
        self.uic.stop_plot_btn.clicked.connect(self.stop)
        self.uic.btn_stop_2.clicked.connect(self.stop_plot)
        self.uic.btn_save_plot.clicked.connect(self.capture_plot)
#Init Graph
        self.graph1 = self.uic.plot_widget2.addGraph()
        self.graph2 = self.uic.plot_widget2.addGraph()
        self.graph3 = self.uic.plot_widget1.addGraph()
        

        pen1 = QPen(QColor(255, 0, 0))
        pen1.setWidth(2) 
        pen2 = QPen(QColor(0, 0, 250))
        pen2.setWidth(2) 
        pen3 = QPen(QColor(3, 255, 50))
        pen3.setWidth(2) 
        self.graph3.setPen(pen3)
        self.graph1.setPen(pen1)
        self.graph2.setPen(pen2)
        #self.graph3.setPen(QPen(QColor(3, 255, 50)))

        scatter_style1 = QCPScatterStyle()
        scatter_style1.setPen(QPen(Qt.NoPen))
        scatter_style1.setBrush(QColor(198, 41, 117))
        # scatter_style1.setSize(5)

        scatter_style2 = QCPScatterStyle()
        scatter_style2.setPen(QPen(Qt.NoPen))
        scatter_style2.setBrush(QColor(198, 41, 117))
        # scatter_style2.setSize(5)

        scatter_style3 = QCPScatterStyle()
        scatter_style3.setPen(QPen(Qt.NoPen))
        scatter_style3.setBrush(QColor(198, 41, 117))
        # scatter_style3.setSize(5)

        self.graph1.setScatterStyle(scatter_style1)
        self.graph2.setScatterStyle(scatter_style2)
        self.graph3.setScatterStyle(scatter_style3)

        self.uic.plot_widget1.setBackground(QBrush(QColor(186, 186, 186)))
        self.uic.plot_widget1.xAxis.setRange(-1, 30)
        self.uic.plot_widget1.yAxis.setRange(-180,180)

        self.uic.plot_widget2.setBackground(QBrush(QColor(186, 186, 186)))
        self.uic.plot_widget2.xAxis.setRange(-1, 30)
        self.uic.plot_widget2.yAxis.setRange(-50,50)

        self.graph1.setData([], [])
        self.graph2.setData([], [])
        self.graph3.setData([], [])

        self.time = 0
        self.a_plot = []
        self.b_plot = []
        self.c_plot = []
        self.t_plot = []

        self.uic.btn_setPID.clicked.connect(self.get_values_PID)
        self.uic.btn_FL.clicked.connect(partial(self.get_single_leg, self.uic.btn_FL))
        self.uic.btn_FR.clicked.connect(partial(self.get_single_leg, self.uic.btn_FR))
        self.uic.btn_BL.clicked.connect(partial(self.get_single_leg, self.uic.btn_BL))
        self.uic.btn_BR.clicked.connect(partial(self.get_single_leg, self.uic.btn_BR))
        self.uic.btn_Set_Angle.clicked.connect(self.set_single_servo)
        self.uic.btn_fw.clicked.connect(self.get_distance_forward)
        self.uic.btn_bw.clicked.connect(self.get_distance_backward)
        self.uic.btn_stand.clicked.connect(self.set_stand)
        self.uic.btn_sit.clicked.connect(self.set_sit)
        self.uic.btn_left.clicked.connect(self.get_step_left)
        self.uic.btn_right.clicked.connect(self.get_step_right)
        self.uic.btn_stop.clicked.connect(self.set_stop)
        self.uic.pushButton.clicked.connect(self.set_emer)

        self.uic.btn_stand.clicked.connect(lambda: self.active_movement(True))
        self.uic.btn_sit.clicked.connect(lambda: self.active_movement(False))
        self.uic.pushButton.clicked.connect(lambda: self.active_movement(False))
        # Gọi hàm cập nhật trạng thái các nút khi khởi động
        self.active_movement(False)

        self.capture_count = 0
        self.plot_count = 0
        self.uic.btn_close_cam.clicked.connect(self.stop_camera)
        self.uic.Start_camera_btn.clicked.connect(self.start_camera)
        self.uic.btn_capture.clicked.connect(self.capture_camera)
    def swap_parts(self, string):
        parts = string.split("_")
        swapped_string = f"{parts[1]}_{parts[0]}"
        return swapped_string
    def start_camera(self):
        # Function to start the camera
        try:
            # Initialize the camera capture
            self.cap = cv2.VideoCapture("http://192.168.43.73:81/stream")  # Adjust the URL as needed

            # Check if camera is opened successfully
            if not self.cap.isOpened():
                raise Exception("Failed to open camera.")

            # Start reading frames from the camera
            while True:
                ret, frame = self.cap.read()

                if ret:
                    
                    # Convert the frame from OpenCV format to QImage
                    height, width, channel = frame.shape
                    bytesPerLine = 3 * width
                    qImg = QImage(frame.data, width, height, bytesPerLine, QImage.Format_RGB888).rgbSwapped()

                    # Display the QImage in the QLabel
                    self.uic.Camera.setPixmap(QPixmap.fromImage(qImg).scaled(self.uic.Camera.size(), Qt.KeepAspectRatio))

                # Wait for a short time to control frame rate
                cv2.waitKey(30)

        except Exception as e:
            print("Error:", e)

    def stop_camera(self):
        # Function to stop the camera
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

    def capture_camera(self):
        # Hàm để chụp ảnh từ camera và lưu vào đường dẫn "D:\" với tên tệp tăng dần
        if hasattr(self, 'cap') and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # Đảm bảo thư mục "D:\" tồn tại
                save_dir = "D:\\"
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)

                # Tạo tên tệp duy nhất cho ảnh
                image_name = f"captured_image_{self.capture_count}.jpg"
                image_path = os.path.join(save_dir, image_name)

                # Lưu ảnh vào đường dẫn "D:\" với tên tệp duy nhất
                cv2.imwrite(image_path, frame)
                print("Image captured and saved successfully at:", image_path)

                # Tăng biến đếm lên để tạo tên tệp duy nhất cho ảnh tiếp theo
                self.capture_count += 1
            else:
                print("Failed to capture image from camera.")
        else:
            print("Camera is not opened.")

    def stop_plot(self):
         self.plot_update_timer.stop()

    def set_last_clicked_button(self, button):
        self.last_clicked_button = button
     
    def get_values_PID(self):
        global Kp,Kd,Ki,mode,roll_ref, pitch_ref,yaw_ref
        mode = "05"
        try:
            # Lấy giá trị từ các Line Text và chuyển đổi thành số thực
            Kp = float(self.uic.Kp.text())
            Kd = float(self.uic.Kd.text())
            Ki = float(self.uic.Ki.text())
            roll_ref = int(self.uic.roll_ref.text())
            pitch_ref = int(self.uic.pitch_ref.text())
            yaw_ref = int(self.uic.yaw_ref.text())
            self.send_message()
        except ValueError:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Vui lòng nhập số thực.")
            msg.setWindowTitle("Lỗi")
            msg.exec_()

    def set_single_servo(self):
        global mode
        mode = "02"
        try:
            
            # for textbox, servo_id in self.textbox_servo_map.items():
            #     if textbox.isModified():
            self.send_message()
        except ValueError:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Vui lòng nhập số thực.")
            msg.setWindowTitle("Lỗi")
            msg.exec_()

    def get_distance_forward(self):
        global distance, mode
        mode = "03"
        try:
        # Lấy giá trị từ các Line Text và chuyển đổi thành số thực
            distance = int(self.uic.distance.text())       
            self.send_message()
        except ValueError:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Vui lòng nhập số thực.")
            msg.setWindowTitle("Lỗi")
            msg.exec_()

    def get_distance_backward(self):
        global distance, mode
        mode = "04"
        try:
        # Lấy giá trị từ các Line Text và chuyển đổi thành số thực
            distance = int(self.uic.distance.text())       
            self.send_message()
        except ValueError:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Vui lòng nhập số thực.")
            msg.setWindowTitle("Lỗi")
            msg.exec_()
    def get_step_left(self):
        global step, mode
        mode = "08"
        try:
        # Lấy giá trị từ các Line Text và chuyển đổi thành số thực
            step = int(self.uic.step.text())       
            self.send_message()
        except ValueError:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Vui lòng nhập số thực.")
            msg.setWindowTitle("Lỗi")
            msg.exec_()
    def get_step_right(self):
        global step, mode
        mode = "09"
        try:
        # Lấy giá trị từ các Line Text và chuyển đổi thành số thực
            step = int(self.uic.step.text())       
            self.send_message()
        except ValueError:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Vui lòng nhập số thực.")
            msg.setWindowTitle("Lỗi")
            msg.exec_()
    def calculate_inverse(self,x, y, z, a1, a2, a3):
        # Constants
        PI = math.pi

        # Calculations
        theta1 = math.atan2(y, x)
        y_23 = y - a1 * math.sin(theta1)
        x_23 = x - a1 * math.cos(theta1)
        L = math.sqrt(x_23**2 + y_23**2 + z**2)
        J3 = math.acos((a2**2 + a3**2 - L**2) / (2 * a2 * a3))
        theta3 = J3 - PI
        B = math.acos((L**2 + a2**2 - a3**2) / (2 * L * a2))
        A = math.atan2(-z, x_23)
        theta2 = B - A
        theta1 = theta1 * 180/PI
        theta2 = theta2 * 180/PI
        theta3 = theta3 * 180/PI
    
        # theta1_Servo = theta1 + theta1_offset
        # theta2_Servo = theta2 + theta2_offset
        # theta3_Servo = theta3 + theta3_offset
      
        # target_x = x;
        # target_y = y;
        # target_z = z;

        return theta1, theta2, theta3
    
    def get_single_leg(self, button):            
        global mode, J1_FL, J2_FL, J3_FL, J1_FR, J2_FR, J3_FR, J1_BL, J2_BL, J3_BL, J1_BR, J2_BR, J3_BR,a1,a2,a3
        mode = "01"
        try:
            J1_FL = float(self.uic.J1_FL.text())
            J2_FL = float(self.uic.J2_FL.text())
            J3_FL = float(self.uic.J3_FL.text())
            theta1_leg1, theta2_leg1, theta3_leg1 = self.calculate_inverse(J1_FL,J2_FL,J3_FL,a1,a2,a3)
            #print(f"Theta1: {theta1_leg1}, Theta2: {theta2_leg1}, Theta3: {theta3_leg1}")
            if -20 <= theta1_leg1 <= 110 and -70 <= theta2_leg1 <= 110 and -138 <= theta3_leg1 <= 42:
                pass
            else:
                raise ValueError("Angle out of range for Inverse Leg ")
            
            J1_FR = float(self.uic.J1_FR.text())
            J2_FR = float(self.uic.J2_FR.text())
            J3_FR = float(self.uic.J3_FR.text())
            theta1_leg2, theta2_leg2, theta3_leg2 = self.calculate_inverse(J1_FR,J2_FR,J3_FR,a1,a2,a3)
            print(f"Theta1: {theta1_leg2}, Theta2: {theta2_leg2}, Theta3: {theta3_leg2}")
            if -18 <= theta1_leg2 <= 112 and -74 <= theta2_leg2 <= 106 and -138 <= theta3_leg2 <= 42:
                pass
            else:
                raise ValueError("Angle out of range for Inverse Leg ")
            
            J1_BL = float(self.uic.J1_BL.text())
            J2_BL = float(self.uic.J2_BL.text())
            J3_BL = float(self.uic.J3_BL.text())
            theta1_leg3, theta2_leg3, theta3_leg3 = self.calculate_inverse(J1_BL,J2_BL,J3_BL,a1,a2,a3)
            #print(f"Theta1: {theta1_leg3}, Theta2: {theta2_leg3}, Theta3: {theta3_leg3}")
            if -113 <= theta1_leg3 <= 17 and -76 <= theta2_leg3 <= 104 and -132 <= theta3_leg3 <= 48:
                pass
            else:
                raise ValueError("Angle out of range for Inverse Leg ")
            
            J1_BR = float(self.uic.J1_BR.text())
            J2_BR = float(self.uic.J2_BR.text())
            J3_BR = float(self.uic.J3_BR.text())
            theta1_leg4, theta2_leg4, theta3_leg4 = self.calculate_inverse(J1_BR,J2_BR,J3_BR,a1,a2,a3)
            #print(f"Theta1: {theta1_leg4}, Theta2: {theta2_leg4}, Theta3: {theta3_leg4}")
            if -109 <= theta1_leg4 <= 21 and -77 <= theta2_leg4 <= 103 and -132 <= theta3_leg4 <= 48:
                pass
            else:
                raise ValueError("Angle out of range for Inverse Leg ")

            self.send_message(button)
        except ValueError as e:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(f"Lỗi: {str(e)}")
            msg.setWindowTitle("Lỗi")
            msg.exec_()
        except Exception as e:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(f"Lỗi: {str(e)}")
            msg.setWindowTitle("Lỗi")
            msg.exec_()
    def set_stand(self):
        global mode
        mode ="06"
        self.send_message()

    def set_sit(self):
        global mode
        mode ="07"
        self.send_message()

    def set_stop(self):
        global mode 
        mode = "10"
        self.send_message()
    def set_emer(self):
        global mode
        mode = "00"
        self.send_message()
    
    def format_number(self, num):
        formatted_num = "{:.3f}".format(num)
        integer_part, fractional_part = formatted_num.split('.')
        if len(integer_part) > 2:
            QMessageBox.warning(self.main_win, "Warning", f"Number {num} exceeds 2 digits before the decimal point.\nToo high control value.", QMessageBox.Ok)
            return None
        padded_integer_part = integer_part.zfill(2)
        return "{}.{}".format(padded_integer_part, fractional_part)
    
    def send_message(self,button = None, servo_id = None):
        global Kp , Kd, Ki, mode, roll_ref, pitch_ref, yaw_ref
        if self.serial_port and self.serial_port.is_open:
            if mode == "05":
                kp_formatted = self.format_number(Kp)
                kd_formatted = self.format_number(Kd)
                ki_formatted = self.format_number(Ki)
                
                if None in [kp_formatted, kd_formatted, ki_formatted]:
                    return None
                message = "{}{}{}{}{:03d}{:03d}{:04d}\r\n".format(
                mode,
                kp_formatted,
                kd_formatted,
                ki_formatted,
                roll_ref,
                pitch_ref,
                yaw_ref
                )
            elif mode == "01":
                if button == self.uic.btn_FL:
                    message = "{}:{:07.2f},{:07.2f},{:07.2f},{}\r\n".format(
                    mode,
                    float(self.uic.J1_FL.text()), 
                    float(self.uic.J2_FL.text()), 
                    float(self.uic.J3_FL.text()),
                    "1"
                )
                elif button == self.uic.btn_FR:
                    message = "{}:{:07.2f},{:07.2f},{:07.2f},{}\r\n".format(
                    mode,
                    float(self.uic.J1_FR.text()), 
                    float(self.uic.J2_FR.text()), 
                    float(self.uic.J3_FR.text()),
                    "4"
                )
                elif button == self.uic.btn_BL:
                    message = "{}:{:07.2f},{:07.2f},{:07.2f},{}\r\n".format(
                    mode,
                    float(self.uic.J1_BL.text()), 
                    float(self.uic.J2_BL.text()), 
                    float(self.uic.J3_BL.text()),
                    "2"
                )
                elif button == self.uic.btn_BR:
                    message = "{}:{:07.2f},{:07.2f},{:07.2f},{}\r\n".format(
                    mode,
                    float(self.uic.J1_BR.text()), 
                    float(self.uic.J2_BR.text()), 
                    float(self.uic.J3_BR.text()),
                    "3"
                )
            elif mode == "02":
                selected_servo = self.uic.combo_IDservo.currentText()
                selected_servo = self.swap_parts(selected_servo)
                for textbox, servo_id in self.textbox_servo_map.items():
                    if selected_servo + "_SV" == textbox.objectName():
                        try:
                            angle = int(textbox.text())
                            if servo_id == 0:
                                if -20 <= angle <= 110:
                                    message = "{}:0{}={:04.0f}\r\n".format(mode, servo_id, angle)
                                else:
                                    raise ValueError("Angle out of range for servo ID {}".format(servo_id))
                            elif servo_id == 1:
                                if -70 <= angle <= 110:
                                    message = "{}:0{}={:04.0f}\r\n".format(mode, servo_id, angle)
                                else:
                                    raise ValueError("Angle out of range for servo ID {}".format(servo_id))
                            elif servo_id == 2:
                                if -138 <= angle <= 42:
                                    message = "{}:0{}={:04.0f}\r\n".format(mode, servo_id, angle)
                                else:
                                    raise ValueError("Angle out of range for servo ID {}".format(servo_id))

                            elif servo_id == 3:
                                if -18 <= angle <= 112:
                                    message = "{}:0{}={:04.0f}\r\n".format(mode, servo_id, angle)
                                else:
                                    raise ValueError("Angle out of range for servo ID {}".format(servo_id))
                            elif servo_id == 4:
                                if -74 <= angle <= 106:
                                    message = "{}:0{}={:04.0f}\r\n".format(mode, servo_id, angle)
                                else:
                                    raise ValueError("Angle out of range for servo ID {}".format(servo_id))
                            elif servo_id == 5:
                                if -138 <= angle <= 42:
                                    message = "{}:0{}={:04.0f}\r\n".format(mode, servo_id, angle)
                                else:
                                    raise ValueError("Angle out of range for servo ID {}".format(servo_id))    
                        
                            elif servo_id == 6:
                                if -113 <= angle <= 17:
                                    message = "{}:0{}={:04.0f}\r\n".format(mode, servo_id, angle)
                                else:
                                    raise ValueError("Angle out of range for servo ID {}".format(servo_id))
                            elif servo_id == 7:
                                if -76 <= angle <= 104:
                                    message = "{}:0{}={:04.0f}\r\n".format(mode, servo_id, angle)
                                else:
                                    raise ValueError("Angle out of range for servo ID {}".format(servo_id))
                            elif servo_id == 8:
                                if -132 <= angle <= 48:
                                    message = "{}:0{}={:04.0f}\r\n".format(mode, servo_id, angle)
                                else:
                                    raise ValueError("Angle out of range for servo ID {}".format(servo_id))  

                            elif servo_id == 9:
                                if -109 <= angle <= 21:
                                    message = "{}:{}={:04.0f}\r\n".format(mode, servo_id, angle)
                                else:
                                    raise ValueError("Angle out of range for servo ID {}".format(servo_id))
                            elif servo_id == 10:
                                if -77 <= angle <= 103:
                                    message = "{}:{}={:04.0f}\r\n".format(mode, servo_id, angle)
                                else:
                                    raise ValueError("Angle out of range for servo ID {}".format(servo_id))
                            elif servo_id == 11:
                                if -132 <= angle <= 48:
                                    message = "{}:{}={:04.0f}\r\n".format(mode, servo_id, angle)
                                else:
                                    raise ValueError("Angle out of range for servo ID {}".format(servo_id))   
                        
                        except ValueError as e:
                            msg = QMessageBox()
                            msg.setIcon(QMessageBox.Warning)
                            msg.setText(f"Lỗi: {str(e)}")
                            msg.setWindowTitle("Lỗi")
                            msg.exec_()
            elif mode == "03":
                message = "{}:{:06d}\r\n".format(mode, distance)
            elif mode == "04":
                message = "{}:{:06d}\r\n".format(mode, distance)
            elif mode == "06":
                message = "{}\r\n".format(mode)
            elif mode == "07":
                message = "{}\r\n".format(mode)    
            elif mode == "08":
                message = "{}:{:02d}\r\n".format(mode, step)
            elif mode == "00":
                message = "{}\r\n".format(mode)
            elif mode == "09":
                message = "{}:{:02d}\r\n".format(mode, step)
            elif mode == "10":
                message = "{}\r\n".format(mode)
            try:
                self.serial_port.write(message.encode())
                print(message)
            except Exception as e:
                QMessageBox.critical(self.main_win, "Error", f"Error sending message: {e}", QMessageBox.Ok)
        else:
            QMessageBox.warning(self.main_win, "Warning", "Serial port not open!", QMessageBox.Ok)

    def active_button(self):
        baudrate_selected = self.uic.comboBox_baudrate.currentText() != "-"
        port_selected = self.uic.comboBox_port.currentText() != "-"
        if baudrate_selected and port_selected:
            self.uic.pushButton_connect.setEnabled(True)
        else:
            self.uic.pushButton_connect.setEnabled(False)


    def active_movement(self, enabled):
        # Gọi hàm để bật hoặc tắt các nút
        self.set_movement_buttons_enabled(enabled)

    def set_movement_buttons_enabled(self, enabled):
        # Hàm này sẽ bật hoặc tắt các nút
        self.uic.btn_setPID.setEnabled(enabled)
        self.uic.btn_FL.setEnabled(enabled)
        self.uic.btn_FR.setEnabled(enabled)
        self.uic.btn_BL.setEnabled(enabled)
        self.uic.btn_BR.setEnabled(enabled)
        self.uic.btn_Set_Angle.setEnabled(enabled)
        self.uic.btn_fw.setEnabled(enabled)
        self.uic.btn_bw.setEnabled(enabled)
        self.uic.btn_left.setEnabled(enabled)
        self.uic.btn_right.setEnabled(enabled)
        self.uic.btn_stop.setEnabled(enabled)


    def init_serial(self):
        self.serial_port = None
        self.serial_reader = SerialReader(self.serial_port)
        self.serial_reader_thread = QThread()
        self.serial_reader.data_received.connect(self.handle_received_data)
        self.serial_reader.moveToThread(self.serial_reader_thread)
        self.serial_reader_thread.started.connect(self.serial_reader.run)

    def connect_serial(self):
        if self.serial_reader_thread.isRunning():
            self.serial_reader.stop()
            self.serial_reader_thread.quit()   
            #self.serial_reader_thread.wait()  # Chờ cho đến khi luồng dừng
            self.serial_port.close()
            self.uic.pushButton_connect.setText("Connect")
  
        else:
            # Get selected COM port and baudrate from combo boxes
            selected_port = self.uic.comboBox_port.currentText()
            selected_baudrate = int(self.uic.comboBox_baudrate.currentText().split("-")[0])
            try:
                # Open the new serial port
                self.serial_port = serial.Serial(
            port=selected_port,
            baudrate=selected_baudrate,
        )
                self.serial_reader.serial_port = self.serial_port
                self.serial_reader_thread.start()
                self.uic.pushButton_connect.setText("Disconnect")
            except Exception as e:
                QMessageBox.critical(self.main_win, "Error", f"Error opening serial port: {e}", QMessageBox.Ok)

    def handle_received_data(self, data):
        global roll_value, pitch_value, yaw_value,Roll_u,Pitch_u
        try:
            global roll_value, pitch_value, yaw_value,Roll_u,Pitch_u
            self.uic.receive_txt.setText(data)
            #yaw_str, pitch_str, roll_str,Pitch_u_str,Roll_u_str = data.split(',')
            yaw_str, pitch_str, roll_str = data.split(',')
        
            # Remove non-numeric characters from the strings
            yaw_str = ''.join(c for c in yaw_str if c.isdigit() or c == '.' or c == '-')
            pitch_str = ''.join(c for c in pitch_str if c.isdigit() or c == '.' or c == '-')
            roll_str = ''.join(c for c in roll_str if c.isdigit() or c == '.' or c == '-')
            #Roll_u_str = ''.join(c for c in roll_str if c.isdigit() or c == '.' or c == '-')
            #Pitch_u_str = ''.join(c for c in roll_str if c.isdigit() or c == '.' or c == '-')

            roll_value = float(roll_str.strip())
            pitch_value = float(pitch_str.strip())
            yaw_value = float(yaw_str.strip())
            #Roll_u = float(Roll_u_str.strip())
            #Pitch_u = float(Pitch_u_str.strip())

            self.uic.roll_txt.setText(str(roll_value))
            self.uic.pitch_txt.setText(str(pitch_value))
            self.uic.yaw_txt.setText(str(yaw_value))

            #print(roll_value, pitch_value, yaw_value,Pitch_u,Roll_u)
            print(roll_value, pitch_value, yaw_value)

            self.roll_value = roll_value
            self.pitch_value = pitch_value 
            self.yaw_value = yaw_value 
        except Exception as e:
            print(f"Error handling received data: {e}")

   

    def plot(self):
        self.plot_update_timer = QTimer()
        self.plot_update_timer.timeout.connect(self.update_plot)
        self.plot_update_timer.start(100)  # Update every 500 milliseconds
        self.uic.plot_widget1.xAxis.setRange(-1, 30)
        self.uic.plot_widget1.yAxis.setRange(-180, 180)
        self.uic.plot_widget2.xAxis.setRange(-1, 30)
        self.uic.plot_widget2.yAxis.setRange(-30, 30)
        

    def stop(self):
        self.clear_plot_data()
        self.plot_update_timer.stop()

    def clear_plot_data(self):
        global t_plot, a_plot, b_plot, c_plot
        t_plot = []
        a_plot = []
        b_plot = []
        c_plot = []

        a_plot.clear()
        b_plot.clear()
        c_plot.clear()
        t_plot.clear()
        self.time = 0
        self.graph1.setData([], [])  # Clear the graph data
        self.graph2.setData([], [])
        self.graph3.setData([], [])
        self.uic.plot_widget1.replot()
        self.uic.plot_widget2.replot()


    def update_plot(self):
        global a_plot, b_plot, t_plot, roll_value, pitch_value, yaw_value, c_plot
    # Thêm giá trị vào danh sách
        a_plot.append(roll_value)
        b_plot.append(pitch_value)
        c_plot.append(yaw_value)
        t_plot.append(self.time)
        self.time += 0.1

    # Cập nhật dữ liệu của đồ thị
        self.graph1.setData(t_plot, a_plot)  # Đặt lại dữ liệu cho đồ thị
        self.graph2.setData(t_plot, b_plot)
        self.graph3.setData(t_plot, c_plot)

       
        self.uic.plot_widget1.xAxis.setRange(0,self.time)
        self.uic.plot_widget1.replot()
        self.uic.plot_widget2.xAxis.setRange(0,self.time)
        self.uic.plot_widget2.replot()

    def capture_plot(self):
       

        filename1 = f"plot_capture_yaw_{self.plot_count}.png"
        filename2 = f"plot_capture_rp_{self.plot_count}.png"


    # Capture the plot
        pixmap1 = self.uic.plot_widget1.grab()
        pixmap2 = self.uic.plot_widget2.grab()

    # Save the captured content as an image file with the unique filename
        pixmap1.save(f"D:/{filename1}")
        pixmap2.save(f"D:/{filename2}")
        self.plot_count =+ 1
    



    def show(self):
         self.main_win.show()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()  # Call show() on the QMainWindow instance
    sys.exit(app.exec_())
