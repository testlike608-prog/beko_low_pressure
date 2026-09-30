import telegram_ask as tel
import db
import client as cl
from cobot_kit import CellError, cycle, robot
import time
from cobot_kit import _fairino, _robot_base

#db.auto_connect_db()
#db.get_product_number(dummy_number=test)
#tel.ask()


class App():
    def __init__(self, scanner_ip, scanner_port):
        self.scannr_ip= scanner_ip
        self.scanner_port= scanner_port
        self.robot_is_connected = False
        #self.robot = robot

        self.OUTPUT_NO   = 0       # الـ DO اللي بيطلع لما الروبوت يوصل النقطة
        self.INPUT_NO    = 0       # الـ DI اللي بنستناه قبل ما نروح للنقطة اللي بعدها
        self.WAIT_S      = 30.0    # أقصى وقت نستنى فيه الـ input (None = للأبد)
        self.APPROACH_MM = 60.0    # يقف قبل النقطة بالمسافة دي على محور الـ tool
        self.CLEARANCE_MM = 0.0    # يقف قبل النقطة نفسها بالمسافة دي (الـ sniffer مش بيلمس)
        self.SPEED       = 20.0    # سرعة الروح للـ approach، %
        self.SLOW        = 8.0     # سرعة آخر حتة ناحية النقطة والرجوع، %
        self.DWELL_S     = 0.5     # يستنى الذراع تهدى قبل الـ output

        pass



    def _set_all_settings(self):
        pass


    
    def _start(self):
        #connect to camera
        #connect to db
        #connect to cobot 
        #connect to scanner client 
        #start lestening for scanner 
        #start the main loop function (_running_loop)
        pass

    def _stop(self):
        #stop camera connection
        #stop the scanner connection
        #stop the main loop function (_running_loop)

        pass


    def _running_loop(self):
        while self.robot_is_connected :
            #check te inpout trigger
            #if the input equals to one start the sequance 
            pass


    
    def _start_sequance(self):
        #get sku from dummy scanned
        #get csv to get number of points
        #move the robot to cap positions 
        #trig the camera to cap images and save it in capture folder
        #send the cap image to the ai model and give me x,y pixel fro image 
        #send the selected welding points from ai to camera to detect the x,y,z for the copot
        #start the cycle function 
        pass
   


    def _robot_cycle(self,points_array):
        
        # start the loop for each point
        # لكل نقطة: approach -> النقطة -> يطلّع output -> يستنى input -> يرجع ورا
        
        

        #robot = self.robot
        orientation = robot.pose().rpy     # الـ tool يفضل بنفس الاتجاه اللي هو عليه دلوقتي
        results = []

        for i, point in enumerate(points_array, 1):
            if not self.robot_is_connected:        # _stop اتنده -- نقف
                break

            pose     = cycle.point_pose(point, orientation, self.CLEARANCE_MM)
            approach = cycle.approach_pose(pose, self.APPROACH_MM)
            print(f"  -> point {i}: {pose}")

            try:
                # 1. روح للنقطة
                robot.move_to(approach, vel=self.SPEED, label=f"point {i} approach")
                robot.move_to(pose, vel=self.SLOW, linear=True, label=f"point {i}")
                time.sleep(self.DWELL_S)

                # 2. طلّع الـ output
                robot.write_output(self.OUTPUT_NO, 1)

                # 3. استنى الـ input
                got_it = robot.wait_input(self.INPUT_NO, 1, timeout_s=self.WAIT_S,
                                          cancel=lambda: not self.robot_is_connected)
                robot.write_output(self.OUTPUT_NO, 0)
                if not got_it:
                    print(f"  !! point {i}: DI{self.INPUT_NO} did not come in {self.WAIT_S} s")

                # 4. ارجع ورا على نفس الخط، وبعدها النقطة اللي بعدها
                robot.move_to(approach, vel=self.SLOW, linear=True, label=f"point {i} retreat")
                results.append({"point": i, "xyz": point.xyz,
                                "ok": got_it, "note": "" if got_it else "input timeout"})

            except CellError as e:
                # نقطة مش reachable أو الكونترولر رفض: سجّلها وكمّل على اللي بعدها
                print(f"  !! point {i} skipped: {e}")
                try:
                    robot.write_output(self.OUTPUT_NO, 0)
                except CellError:
                    pass
                results.append({"point": i, "xyz": point.xyz, "ok": False, "note": str(e)})

        return results








##################################################################