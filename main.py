import cv2, mediapipe as mp, math, time
from djitellopy import Tello
import threading
from queue import Queue


class Frontend(object):

    def __init__(self):
        self.cv = cv2
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.drone_action_queue = Queue()
        self.drone_thread = threading.Thread(target=self.drone_action_worker)
        self.drone_thread.start()
        self.tl = Tello()

    def run(self):
        self.tl.connect()
        self.tl.streamon()
        # cap = self.cv.VideoCapture(self.tl.get_udp_video_address())
        cap = self.cv.VideoCapture(0)
        
        with self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
        ) as hands:
            while cap.isOpened():

                ret, frame = cap.read()
                if not ret:
                    break

                # RGR to RGB
                image = self.cv.cvtColor(frame, self.cv.COLOR_BGR2RGB)

                # detections
                # image.flags.writeable = False

                # set flag true
                results = hands.process(image)
                # image.flags.writeable = True

                # # recolor
                # image = self.cv.cvtColor(image, self.cv.COLOR_BGR2RGB)
                # print(results)
                """
                x= horizontal
                y= vertical
                z= depth
                
                """

                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.mp_drawing.draw_landmarks(
                            frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                        )

                        # landmarks = hand_landmarks.landmark

                        landmarks = [(lm.x, lm.y) for lm in hand_landmarks.landmark]

                        gesture = self.reconocerGestos(landmarks)

                    if gesture == "thumb_up":
                        self.printGestureOutput(gesture, frame)
                        self.drone_action_queue.put("takeoff")
                    elif gesture == "ok":
                        self.printGestureOutput(gesture, frame)
                        self.drone_action_queue.put("land")
                    elif gesture == "index_up":
                        self.printGestureOutput(gesture, frame)
                        self.drone_action_queue.put("up")
                    elif gesture == "index_down":
                        self.printGestureOutput(gesture, frame)
                        self.drone_action_queue.put("down")
                    elif gesture == "index_right":
                        self.printGestureOutput(gesture, frame)
                        self.drone_action_queue.put("right")
                    elif gesture == "index_left":
                        self.printGestureOutput(gesture, frame)
                        self.drone_action_queue.put("left")
                    elif gesture == "palm":
                        self.printGestureOutput(gesture, frame)
                        self.drone_action_queue.put("forward")
                    elif gesture == "rock":
                        self.printGestureOutput(gesture, frame)
                        self.drone_action_queue.put("back")
                    elif gesture =="":
                        self.printGestureOutput(gesture, frame)
                        self.drone_action_queue.put(None)
                        

                    for num, hand in enumerate(results.multi_hand_landmarks):
                        self.mp_drawing.draw_landmarks(
                            frame,
                            hand,
                            self.mp_hands.HAND_CONNECTIONS,
                            self.mp_drawing.DrawingSpec(
                                color=(121, 22, 76), thickness=2, circle_radius=4
                            ),
                            self.mp_drawing.DrawingSpec(
                                color=(121, 44, 250), thickness=2, circle_radius=2
                            ),
                        )

                    # for hand_landmarks in results.multi_hand_landmarks:
                    #     for id, landmark in enumerate(hand_landmarks.landmark):
                    #         print(
                    #             f"Landmark {id}: x={landmark.x}, y={landmark.y}, z={landmark.z}"
                    #         )

                self.cv.imshow("Hand tracking", frame)

                if self.cv.waitKey(1) & 0xFF == ord("q"):
                    break
            self.tl.end()

        self.cv.destroyAllWindows()
        self.drone_action_queue.put(None)  # Señal para terminar el hilo
        self.drone_thread.join()

    def reconocerGestos(self, landmarks):
        # muñeca
        munieca = landmarks[0]

        # pulgar
        pulgar01, pulgar02, pulgar03, pulgar04 = (
            landmarks[1],
            landmarks[2],
            landmarks[3],
            landmarks[4],
        )

        # indice
        indice01 = landmarks[5]
        indice02 = landmarks[6]
        indice03 = landmarks[7]
        indice04 = landmarks[8]

        # medio
        medio01 = landmarks[9]
        medio02 = landmarks[10]
        medio03 = landmarks[11]
        medio04 = landmarks[12]

        # anular
        anular01 = landmarks[13]
        anular02 = landmarks[14]
        anular03 = landmarks[15]
        anular04 = landmarks[16]

        # meñique
        meniique01 = landmarks[17]
        meniique02 = landmarks[18]
        meniique03 = landmarks[19]
        meniique04 = landmarks[20]

        # is_palm = all(
        #     landmarks[base][1] > landmarks[punta][1]
        #     for base, punta in [(1, 4), (5, 8), (9, 12), (13, 16), (17, 20)]
        # )

        is_ok = (
            self.distance(pulgar04, indice04) <= 0.03 and self.distance(pulgar04, medio04) >0.05
            # and all(
            #     landmarks[base][1] > landmarks[punta][1]
            #     for base, punta in [(9, 12), (13, 16), (17, 20)]
            # )
            # or (pulgar01[1] < pulgar04[1] and indice01[1] < pulgar04[1])
        )

        is_thumb_up = (
            all(
                landmarks[base][1] < landmarks[punta][1]
                for base, punta in [(5, 8), (9, 12), (13, 16), (17, 20)]
            )
            and pulgar01[1] > pulgar04[1]
        )

        is_index_up= (
            munieca[1]>
            indice02[1]>indice04[1]
            and
            #medio to meñique
            all(
                landmarks[base][1]< landmarks[punta][1]
                for base, punta in [(9,12), (13, 16), (17, 20)]
            )
            # or
            # pulgar02[1]<pulgar04[1]
            
        )
        is_index_down= (
            munieca[1]<
            (indice02[1]<indice04[1])
            and
            #medio to meñique
            all(
                landmarks[base][1]< landmarks[punta][1]
                for base, punta in [(9,12), (13, 16), (17, 20)]
            )
            # or
            # pulgar02[1]<pulgar04[1]
            
        )
        
        is_index_right=(
            indice04[0]>indice02[0]
            and
            (all (
                landmarks[base][1]<landmarks[punta][1]
                for base, punta in [(2,4), (10,12), (14,16), (18,20)]
            ))
        )
        
        is_index_left=(
            indice04[0]<indice02[0]
            and
            (all (
                landmarks[base][1]<landmarks[punta][1]
                for base, punta in [(2,4), (10,12), (14,16), (18,20)]
            ))
        )
        
        is_palm = ( 
            all(landmarks[base][1]>landmarks[punta][1]
                for base, punta in [(1,4), (5,8), (9,12), (13,16), (17,20)])           
        )
        
        is_rock= (
            indice02[1]>indice04[1]
            and
            meniique02[1]>meniique04[1]
            
            and
            (all(
                landmarks[base][1]<landmarks[punta][1]
                for base, punta in [(10,12), (14,16)]
            ))
            and
            pulgar01[1]>pulgar04[1]
        )
        """
        0: Base de la palma (wrist, muñeca).
        1-4: Pulgar (base, primera articulación, segunda articulación y punta).
        5-8: Índice (base, primera articulación, segunda articulación y punta).
        9-12: Medio.
        13-16: Anular.
        17-20: Meñique.
        """

        #land
        if is_ok:
            return "ok"
        #takeoff
        elif is_thumb_up:
            return "thumb_up"
        #rotate right
        elif is_index_right:
            return "index_right"
        #rotate left
        elif is_index_left:
            return "index_left"
        #move up
        elif is_index_up:
            return "index_up"
        #move down
        elif is_index_down:
            return "index_down"
        #move forward
        elif is_palm:
            return "palm"
        #move back
        elif is_rock:
            return "rock"
        else:
            return "unknown"

    def distance(self, dot1, dot2):
        return math.sqrt((dot1[0] - dot2[0]) ** 2 + (dot1[1] - dot2[1]) ** 2)

    def printGestureOutput(self, gesture, frame):
        if gesture != "unknown":
            self.cv.putText(
                frame,
                f"{gesture}",
                (50, 50),
                self.cv.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )
        else:
            self.cv.putText(
                frame,
                f"{gesture}",
                (50, 50),
                self.cv.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
            )
    """
    
        to-do: que capture el gesto solo una vez Y
        que ese gesto sea detectado dentro de un plazo de 1 a 2 segundos
        
    """
    def drone_action_worker(self):
        gesture_detected= False
        while True:
            action = self.drone_action_queue.get()
            if action is None:  # Señal para terminar el hilo
                break
            try:
                if action == "takeoff":
                    """if not gesture_detected: 
                        self.tl.takeoff()
                        # print("al vuelo")
                        gesture_detected= True"""
                elif action == "land":
                    """if not gesture_detected: 
                        self.tl.land()
                        # print("al vuelo")
                        gesture_detected= True"""
                elif action == "up":
                    """if not gesture_detected: 
                        self.tl.move_up(20)
                        # print("al vuelo")
                        gesture_detected= True"""
                elif action == "down":
                    """if not gesture_detected: 
                        self.tl.move_down(20)
                        # print("al vuelo")
                        gesture_detected= True"""
                elif action == "right":
                    """if not gesture_detected: 
                        self.tl.rotate_clockwise(45)
                        # print("al vuelo")
                        gesture_detected= True"""
                elif action == "left":
                    """if not gesture_detected: 
                        self.tl.rotate_counter_clockwise(45)
                        # print("al vuelo")
                        gesture_detected= True"""
                elif action == "forward":
                    """if not gesture_detected: 
                        self.tl.move_forward(20)
                        # print("al vuelo")
                        gesture_detected= True"""
                elif action == "back":
                    """if not gesture_detected: 
                        self.tl.move_back(20)
                        # print("al vuelo")
                        gesture_detected= True"""
                    
            except Exception as e:
                print(f"Error ejecutando acción '{action}': {e}")
                pass
            self.drone_action_queue.task_done()

def main():
    fend = Frontend()
    fend.run()

if __name__ == "__main__":
    main()
