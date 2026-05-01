import math
from controller import Robot

class AutonomousMavic:
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # تعريف الحساسات
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.time_step)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.time_step)
        
        # تعريف المحركات
        self.motors = [self.robot.getDevice(n) for n in ["front left propeller", "front right propeller", "rear left propeller", "rear right propeller"]]
        for m in self.motors:
            m.setPosition(float('inf'))
            m.setVelocity(0)

        # معاملات التحكم PID
        self.k_roll_p, self.k_roll_d = 30.0, 2.0
        self.k_pitch_p, self.k_pitch_d = 30.0, 2.0
        self.k_yaw_p, self.k_yaw_d = 2.0, 1.0 

        # --- مصفوفة النقاط (نقطتين فقط) ---
        self.waypoints = [
            [-4.32, -3.99],      
            [4.32, -3.99],  
            [-4.32, 3.99], 
            [4.32, 3.99]  
        ]
        self.current_idx = 0
        
        self.target_altitude = 2.0
        self.prev_altitude = 0.0
        self.initial_yaw = None 
        
        self.state = 0
        self.home = None 
        self.hover_timer = 0 
        self.is_returning = False
        
        self.MAX_FORWARD_PITCH = 0.1 

    def run(self):
        while self.robot.step(self.time_step) != -1:
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            roll_rate, pitch_rate, yaw_rate = self.gyro.getValues()
            curr_x, curr_y, curr_z = self.gps.getValues()

            if self.initial_yaw is None: 
                self.initial_yaw = yaw

            # تحديد الهدف: إما المنزل أو النقطة الحالية من المصفوفة
            if self.is_returning:
                target_x, target_y = self.home
            else:
                target_x, target_y = self.waypoints[self.current_idx]
            
            dist = math.sqrt((target_x - curr_x)**2 + (target_y - curr_y)**2)
            
            desired_pitch = 0.0
            desired_roll = 0.0
            target_yaw = self.initial_yaw 

            # --- منطق الحالات المعدل ليتناسب مع نقطتين ---
            
            if self.state == 0: # الإقلاع
                if self.home is None:
                    self.home = [curr_x, curr_y]
                    print("🚀 إقلاع...")
                if abs(self.target_altitude - curr_z) < 0.1:
                    self.state = 1
            
            elif self.state == 1: # الدوران نحو الهدف الحالي
                target_yaw = math.atan2(target_y - curr_y, target_x - curr_x)
                yaw_error = target_yaw - yaw
                while yaw_error > math.pi: yaw_error -= 2.0 * math.pi
                while yaw_error < -math.pi: yaw_error += 2.0 * math.pi
                if abs(yaw_error) < 0.1:
                    self.state = 2
                    
            elif self.state == 2: # الطيران نحو الهدف
                target_yaw = math.atan2(target_y - curr_y, target_x - curr_x)
                speed_factor = min(dist / 3.0, 1.0) 
                desired_pitch = self.MAX_FORWARD_PITCH * speed_factor 
                
                if dist < 0.6: 
                    print(f"📍 وصلنا للنقطة رقم {self.current_idx + 1}. جاري التثبيت...")
                    self.state = 3
            
            elif self.state == 3: # الفرملة والاستقرار
                if self.hover_timer < 50:
                    desired_pitch = -0.06 
                else:
                    desired_pitch = 0.0
                
                self.hover_timer += 1
                if self.hover_timer > 100:
                    self.hover_timer = 0
                    
                    if not self.is_returning:
                        # إذا كان هناك نقاط متبقية في المصفوفة
                        if self.current_idx < len(self.waypoints) - 1:
                            self.current_idx += 1
                            print(f"➡️ التوجه للنقطة التالية: {self.current_idx + 1}")
                            self.state = 1 # ارجع للدوران للهدف الجديد
                        else:
                            # انتهت النقاط، حان وقت العودة
                            print("🏠 تمت زيارة كل النقاط. العودة للمنزل...")
                            self.is_returning = True
                            self.state = 1
                    else:
                        print("🛬 فوق المنزل. بدء الهبوط...")
                        self.state = 4
                            
            elif self.state == 4: # الهبوط
                self.target_altitude -= 0.005 
                if curr_z < 0.15: 
                    print("🏁 هبوط آمن!")
                    self.state = 5
                    
            elif self.state == 5:
                for motor in self.motors: motor.setVelocity(0)
                continue

            # --- نظام التحكم (الخلاط) ---
            yaw_error = target_yaw - yaw
            while yaw_error > math.pi: yaw_error -= 2.0 * math.pi
            while yaw_error < -math.pi: yaw_error += 2.0 * math.pi

            v_vel = (curr_z - self.prev_altitude) / (self.time_step / 1000.0)
            self.prev_altitude = curr_z
            vertical_input = 68.5 + (min(max(self.target_altitude - curr_z, -1.0), 1.0) * 10.0) - (v_vel * 5.0)
            
            roll_input = (desired_roll - roll) * self.k_roll_p - (roll_rate * self.k_roll_d)
            pitch_input = (desired_pitch - pitch) * self.k_pitch_p - (pitch_rate * self.k_pitch_d)
            yaw_input = (yaw_error * self.k_yaw_p) - (yaw_rate * self.k_yaw_d)

            m1 = vertical_input + roll_input - pitch_input - yaw_input
            m2 = vertical_input - roll_input - pitch_input + yaw_input
            m3 = vertical_input + roll_input + pitch_input + yaw_input
            m4 = vertical_input - roll_input + pitch_input - yaw_input
            
            self.motors[0].setVelocity(m1)
            self.motors[1].setVelocity(-m2)
            self.motors[2].setVelocity(-m3)
            self.motors[3].setVelocity(m4)

controller = AutonomousMavic()
controller.run()