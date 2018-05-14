import zmq

class SampleListener:
    context = None
    zmq_socket = None
    def on_init(self):
        print("\nInitialized")
        self.context = zmq.Context()
        self.zmq_socket = self.context.socket(zmq.PUSH)
        self.zmq_socket.bind("tcp://127.0.0.1:5557") 

    def on_frame(self):
        print("\nOn Frame")
        a = b = c = 1
        work_message = str(a)+","+str(b)+","+str(c)+'\n'
        self.zmq_socket.send_json(work_message)

def main():
    # Create a sample listener and controller
    listener = SampleListener()
    listener.on_init()
    for i in range(100):
        listener.on_frame()

if __name__ == "__main__":
    main()


# import os, sys, thread, time
# import Leap
# import csv
# import zmq 
# from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

# def write_csv(data):
#     with open('output.csv','a') as fp:
#         a = csv.writer(fp)
#         a.writerows(data)


   
# class SampleListener(Leap.Listener):
#     finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
#     bone_names = ['Metarpal', 'Proximal', 'Intermediate', 'Distal']
#     state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

#     context = None
#     zmq_socket = None
#     count = 0
#     alpha = 0.5
#     min_angle_PP_MC_index = 180
#     max_angle_PP_MC_index = 0 
#     min_angle_MP_PP_index = 180
#     max_angle_MP_PP_index = 0       
#     min_angle_DP_MP_index = 180
#     max_angle_DP_MP_index = 0
#     angle_PP_MC_index_1 = 0
#     angle_PP_MC_index_0 = 0
#     angle_MP_PP_index_1 = 0
#     angle_MP_PP_index_0 = 0
#     angle_DP_MP_index_1 = 0
#     angle_DP_MP_index_0 = 0
#     def on_init(self, controller):
#         print "\nInitialized"
#         self.context = zmq.Context()
#         self.zmq_socket = self.context.socket(zmq.PUSH)
#         self.zmq_socket.bind("tcp://127.0.0.1:5557") 
#         self.data=[0,0,0]
    
#     def on_connect(self, controller):
#         print "\nMotion Sensor Connected"
#         controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
#         controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
#         controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
#         controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

#     def on_disconnect(self, controller):
#         print "\nMotion Sensor Disconnected"

#     def on_exit(self, controller):
#         print "Exit!"

#     def on_frame(self, controller):
#         ##ser = serial.Serial('COM4',1000000)
#         frame = controller.frame()

#         for hand in frame.hands:
            
#             print "\ncount: ", SampleListener.count     
#             index_finger_list = hand.fingers.finger_type(Leap.Finger.TYPE_INDEX)
#             index_finger = index_finger_list[0] #since there is only one per hand

#             metacarpal_index = index_finger.bone(Leap.Bone.TYPE_METACARPAL)
#             direction_metacarpal_index = metacarpal_index.direction
#    ##         print direction_metacarpal_index

# ## Angle of Proximal to Metacarpal
#             proximal_index = index_finger.bone(Leap.Bone.TYPE_PROXIMAL)
#             direction_proximal_index = proximal_index.direction
#   ##          print direction_proximal_index
#             SampleListener.angle_PP_MC_index_1 = (direction_proximal_index.angle_to(direction_metacarpal_index) / 3.14 * 180)
#             if (SampleListener.angle_PP_MC_index_1 < SampleListener.min_angle_PP_MC_index):
#                 SampleListener.min_angle_PP_MC_index = SampleListener.angle_PP_MC_index_1
#             if (SampleListener.angle_PP_MC_index_1 > SampleListener.max_angle_PP_MC_index):
#                 SampleListener.max_angle_PP_MC_index = SampleListener.angle_PP_MC_index_1

# ## Angle of Intermediate to Proximal
#             intermediate_index = index_finger.bone(Leap.Bone.TYPE_INTERMEDIATE)
#             direction_intermediate_index = intermediate_index.direction
#   ##          print direction_proximal_index
#             SampleListener.angle_MP_PP_index_1 = (direction_intermediate_index.angle_to(direction_proximal_index) / 3.14 * 180)
#             if (SampleListener.angle_MP_PP_index_1 < SampleListener.min_angle_MP_PP_index):
#                 SampleListener.min_angle_MP_PP_index = SampleListener.angle_MP_PP_index_1
#             if (SampleListener.angle_MP_PP_index_1 > SampleListener.max_angle_MP_PP_index):
#                 SampleListener.max_angle_MP_PP_index = SampleListener.angle_MP_PP_index_1

# ## Angle of Distal to Intermediate
#             distal_index = index_finger.bone(Leap.Bone.TYPE_DISTAL)
#             direction_distal_index = distal_index.direction
#  ##           print direction_distal_index
#             SampleListener.angle_DP_MP_index_1 = (direction_distal_index.angle_to(direction_intermediate_index) / 3.14 * 180)
#             if (SampleListener.angle_DP_MP_index_1 < SampleListener.min_angle_DP_MP_index):
#                 SampleListener.min_angle_DP_MP_index = SampleListener.angle_DP_MP_index_1
#             if (SampleListener.angle_DP_MP_index_1 > SampleListener.max_angle_DP_MP_index):
#                 SampleListener.max_angle_DP_MP_index = SampleListener.angle_DP_MP_index_1


# ## Send index to Servo

#             ##This segment is to filter noises with alpha index:
#             angle_PP_MC_index = (SampleListener.alpha * SampleListener.angle_PP_MC_index_1 + (1 - SampleListener.alpha) * SampleListener.angle_PP_MC_index_0)
#             angle_MP_PP_index = (SampleListener.alpha * SampleListener.angle_MP_PP_index_1 + (1 - SampleListener.alpha) * SampleListener.angle_MP_PP_index_0)
#             angle_DP_MP_index = (SampleListener.alpha * SampleListener.angle_DP_MP_index_1 + (1 - SampleListener.alpha) * SampleListener.angle_DP_MP_index_0)
            

#             ##print "PP_MC: " + str(round(angle_PP_MC_index, 2)) + " MP_PP: " + str(round(angle_MP_PP_index, 2)) + " DP_MP: " + str(round(angle_DP_MP_index, 2))
#             a = round(angle_PP_MC_index*1.15, 2)
#             b = round(angle_MP_PP_index, 2)
#             c = round(angle_DP_MP_index, 2)
#             self.data=[a,b,c]
#             print self.data

# ## Writing 3 angle values to csv and txt
#             fd = open('data_leapmotion.txt','a')
#             fd.write(str(a)+","+str(b)+","+str(c)+'\n')
#             fd.close()
#             '''
#             with open('data_leapmotion.txt', 'rb') as f_data, open('data_leapmotion.csv', 'wb') as f_output:
#                 csv_data = csv.reader(f_data)
#                 csv_output = csv.writer(f_output)
#                 csv_output.writerow(['PP_MC', 'MP_PP', 'DP_MP'])
#                 for row in csv_data:
#                 csv_output.writerow(row) 
#              '''

#             # Start your result manager and workers before you start your producer        
#             work_message = str(a)+","+str(b)+","+str(c)+'\n'
#             self.zmq_socket.send_json(work_message)

#             time.sleep(0.05)
#             SampleListener.angle_PP_MC_index_0 = SampleListener.angle_PP_MC_index_1
#             SampleListener.angle_MP_PP_index_0 = SampleListener.angle_MP_PP_index_1
#             SampleListener.angle_DP_MP_index_0 = SampleListener.angle_DP_MP_index_1
#             SampleListener.count += 1
                   
# def main():
#     # Create a sample listener and controller
#     listener = SampleListener()
#     controller = Leap.Controller() 
#     # Have the sample listener receive events from the controller
#     controller.add_listener(listener)

    
#     # Keep this process running until Enter is pressed
#     print "\nPress Enter to quit..."
#     try:
#         sys.stdin.readline()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # Remove the sample listener when done
#         controller.remove_listener(listener)
  

# if __name__ == "__main__":
#     main()