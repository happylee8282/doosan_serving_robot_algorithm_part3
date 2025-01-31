from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QListWidget, QListWidgetItem, QFrame, QPushButton, QSpinBox, QMessageBox

#openai와 gui제작
from PyQt5.QtGui import QPixmap
import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QListWidget, QMessageBox, QSpinBox, QListWidgetItem
)
from PyQt5.QtCore import QTimer
import openai
import pyaudio
import wave
import speech_recognition as sr
import numpy as np
import time
import sys
import rclpy
import threading
import signal
import pyttsx3  # pyttsx3 라이브러리 임포트
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from gtts import gTTS
import os
import playsound
#--------------------------------------------------------------------------
#노드관련
import rclpy
import threading
import signal
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile
import queue
import json
from food_msg.srv import Foodcustomer
import array
from datetime import date
import sqlite3
#--------------------------------------------------------------------------
#Node publish
class NODE(Node):
    def __init__(self):
        super().__init__('burger_kiosk_operator')
        # 서비스 클라이언트 생성
        self.client = self.create_client(Foodcustomer, 'pay_response')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')

        conn = sqlite3.connect('/home/happy/Desktop/menu.db')
        cursor = conn.cursor()

        order = 'Select * from menu'

        cursor.execute(order)

        result = cursor.fetchall()

        conn.commit()
        conn.close()

        ff = []
        mm = []

        for i in result:
            ff.append(i[0])
            mm.append(i[2])


        self.declare_parameter('food_ids', ff)
        self.declare_parameter('money', mm)

        self.food = self.get_parameter('food_ids')
        self.money = self.get_parameter('money')
    
    def send_order(self, table_num, food_ids, food_counts):
        # 서비스 요청 메시지 생성
        request = Foodcustomer.Request()
        request.table_num = table_num
        request.food_id = food_ids
        request.food_count = food_counts

        # 비동기 서비스 호출
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)
    
    def handle_response(self, future):
        try:
            response = future.result()
            if response.success == 1:
                self.get_logger().info("Order processed successfully!")
            else:
                self.get_logger().warn("Order processing failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}") 
            #logger_error
            #서비스가 연결이 안되었을떄 error를 표시함으로써 service상에서 문제가 있다는 것을 발견 

#--------------------------------------------------------------------------
#OPENAI PART
# OpenAI API 키 설정
key = "api_key"
client = openai.OpenAI(api_key=key)

# TTS 엔진 초기화
tts_engine = pyttsx3.init(driverName='espeak')  # espeak 엔진 사용
tts_engine.setProperty('rate', 140)  # 음성 속도 조정
tts_engine.setProperty('volume', 0.9)  # 볼륨 최대 설정
tts_engine.setProperty('voice', 'ko-KR')

# 녹음 설정
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000               # 샘플링 속도
FRAMES_PER_BUFFER = 2048   # 버퍼 크기
SILENCE_DURATION = 1  # 초기 침묵 감지 시간은 1초

# PyAudio 객체 초기화
p = pyaudio.PyAudio()

# 음성 인식 실패 횟수를 추적하는 변수
failed_recognition_count = 0

def save_audio(frames, filename):
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()


recognizer = sr.Recognizer()

# 음성 파일을 텍스트로 변환 후 OpenAI API와 대화
history_messages = [
    {"role": "system", "content": "당신의 이름은 철수입니다. 당신은 활기찬 햄버거 알바생이고 햄버거를 정말 좋아압니다."}
]

def recognize_audio(filename):
    with sr.AudioFile(filename) as source:
        audio = recognizer.record(source)
        try:
            text = recognizer.recognize_google(audio, language="ko-KR")
            return text
        except sr.UnknownValueError:
            return None
        except sr.RequestError:
            return None


def record_audio(silence_duration):
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=FRAMES_PER_BUFFER)

    frames = []
    silence_start = None  # 침묵 시작 시간

    while True:
        data = stream.read(FRAMES_PER_BUFFER)
        frames.append(data)

        # 데이터가 16비트 정수로 변환
        audio_data = np.frombuffer(data, dtype=np.int16)

        # 평균 볼륨 계산 (신호 강도)
        volume = np.abs(audio_data).mean()

        if volume < 500:  # 볼륨이 낮은 경우 침묵으로 간주
            if silence_start is None:
                silence_start = time.time()  # 침묵 시작 시간 기록
        else:
            silence_start = None  # 음성이 감지되면 침묵 시작 시간 초기화

        # 침묵 시간이 설정된 시간(silence_duration) 이상이면 종료
        if silence_start is not None and (time.time() - silence_start) > silence_duration:
            break

    # 스트림 종료
    stream.stop_stream()
    stream.close()

    return frames

waiting_for_response = False

class RecordAudioWorker(QThread):
    recording_done = pyqtSignal(list)  # 녹음 결과를 전달할 신호
    
    def __init__(self, silence_duration):
        super().__init__()
        self.silence_duration = silence_duration

    def run(self):
        frames = record_audio(self.silence_duration)
        self.recording_done.emit(frames)  # 녹음 결과 전달

class TTSWorker(QThread):
    def __init__(self, text):
        super().__init__()
        self.text = text

    def run(self):
        try:
            tts = gTTS(text=self.text, lang='ko')
            tts.save("response.mp3")
            playsound.playsound("response.mp3")
            os.remove("response.mp3")  # 사용 후 삭제
        except Exception as e:
            print(f"TTS 오류 발생: {e}")

# GPTWorker 클래스 정의
class GPTWorker(QThread):
    response_ready = pyqtSignal(str)  # 응답 신호
    error_occurred = pyqtSignal(str)  # 오류 신호
    
    def __init__(self, audio_file, history_messages, client):
        super().__init__()
        self.audio_file = audio_file
        self.history_messages = history_messages
        self.client = client

    def run(self):
        try:
            # 음성을 텍스트로 변환
            text = recognize_audio(self.audio_file)
            if not text:
                self.error_occurred.emit("음성을 인식할 수 없습니다.")
                return

            # OpenAI API 호출
            self.history_messages.append({"role": "user", "content": text})
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=self.history_messages,
                temperature=0.2
            )
            answer = response.choices[0].message.content
            self.response_ready.emit(answer)  # 응답 신호 방출
            self.history_messages.append({"role": "assistant", "content": answer})

        except Exception as e:
            self.error_occurred.emit(f"GPT 호출 중 문제가 발생했습니다: {str(e)}")

#--------------------------------------------------------------------------
#customer_gui
class BurgerKiosk(QWidget):
    def __init__(self,node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Burger Kiosk_5") #키오스크
        self.setGeometry(100, 100, 600, 550)

        # Main layout
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        # Category buttons layout
        self.category_layout = QHBoxLayout()
        self.main_layout.addLayout(self.category_layout)

        self.category_buttons = {
            "Burgers": QLabel("Burgers"),
            "Side": QLabel("Side"),
            "Drinks": QLabel("Drinks")
        }
        
        for category, label in self.category_buttons.items():
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("font-size: 16px; font-weight: bold; padding: 10px; border: 1px solid black;")
            label.mousePressEvent = lambda event, cat=category: self.show_menu(cat)
            self.category_layout.addWidget(label)
        
        # Menu layout
        self.menu_layout = QGridLayout()
        self.main_layout.addLayout(self.menu_layout)

        # Order list
        self.order_list = QListWidget()
        self.main_layout.addWidget(self.order_list)

        # Total price
        self.total_price_label = QLabel("Total: $0.00")
        self.main_layout.addWidget(self.total_price_label)

        # Action buttons layout
        self.action_layout = QHBoxLayout()
        self.pay_button = QPushButton("Pay")
        self.pay_button.clicked.connect(self.pay_order)
        self.clear_button = QPushButton("Clear")
        self.clear_button.clicked.connect(self.clear_order)
        self.gpt_button = QPushButton("GPT")
        self.gpt_button.clicked.connect(self.say_gpt)
        
        self.action_layout.addWidget(self.pay_button)
        self.action_layout.addWidget(self.clear_button)
        self.action_layout.addWidget(self.gpt_button)
        self.main_layout.addLayout(self.action_layout)

        food_list = self.node.food.to_parameter_msg().value.string_array_value
        money_list = self.node.money.to_parameter_msg().value.integer_array_value
        # Menu data with absolute paths

        print(f"hi{food_list}")
        print(f"hi{money_list}")

        self.menus = {
            "Burgers": [(food_list[0], "/home/happy/Desktop/newfile/cheese.jpeg", money_list[0]),
                         (food_list[1], "/home/happy/Desktop/newfile/chicken.jpeg", money_list[1]),
                         (food_list[2], "/home/happy/Desktop/newfile/veggie.jpeg", money_list[2])],
            "Side": [(food_list[3], "/home/happy/Desktop/newfile/fries.jpeg", money_list[3]),
                      (food_list[4], "/home/happy/Desktop/newfile/onion.jpeg", money_list[4]),
                      (food_list[5], "/home/happy/Desktop/newfile/nuggert.jpeg", money_list[5])],
            "Drinks": [(food_list[6], "/home/happy/Desktop/newfile/colo.jpeg", money_list[6]),
                        (food_list[7], "/home/happy/Desktop/newfile/sprite.jpeg", money_list[7]),
                        (food_list[8], "/home/happy/Desktop/newfile/sam.jpeg", money_list[8])]
        }
        self.selected_order = {}

    def pay_order(self):
        if not self.selected_order:
            QMessageBox.warning(self, "Error", "No items in your order.")
            self.get_logger().warn("No items in your order.")
            return

        table_number = 5  #num_table
        food_ids = [item for item in self.selected_order.keys()]
        food_counts_array =  array.array('i', [details["quantity"] for details in self.selected_order.values()])
        food_counts = list(food_counts_array)  # array -> list 변환
        
        # NODE의 send_order 호출
        self.node.send_order(table_number, food_ids, food_counts)

        order_summary = "\n".join([f"{item} x{details['quantity']}" for item, details in self.selected_order.items()])
        total_price = sum(details['quantity'] * details['price'] for details in self.selected_order.values())
        QMessageBox.information(self, "Order Placed", f"Your order has been placed!\n{order_summary}\nTotal: ${total_price}")
        self.clear_order()

    def show_menu(self, category):
        for i in reversed(range(self.menu_layout.count())):
            widget = self.menu_layout.itemAt(i).widget()
            if widget is not None:
                widget.deleteLater()

        row, col = 0, 0
        for item, image, price in self.menus[category]:
            frame = QFrame()
            frame.setStyleSheet("border: 1px solid black; padding: 5px;")
            item_layout = QVBoxLayout(frame)
            
            pixmap = QPixmap(image).scaled(100, 100, Qt.KeepAspectRatio)
            image_label = QLabel()
            image_label.setPixmap(pixmap)
            image_label.setAlignment(Qt.AlignCenter)
            image_label.mousePressEvent = lambda event, i=item, p=price: self.add_to_order(i, p)
            
            text_label = QLabel(f"{item}\n${price}")
            text_label.setAlignment(Qt.AlignCenter)
            
            item_layout.addWidget(image_label)
            item_layout.addWidget(text_label)
            frame.setLayout(item_layout)
            
            self.menu_layout.addWidget(frame, row, col)
            col += 1
            if col > 2:
                col = 0
                row += 1

    def add_to_order(self, item, price):
        if item in self.selected_order:
            self.selected_order[item]['quantity'] += 1
        else:
            self.selected_order[item] = {'price': price, 'quantity': 1}
        self.update_order_list()

    def update_order_list(self):
        self.order_list.clear()
        total_cost = 0
        for item, details in self.selected_order.items():
            quantity = details['quantity']
            price = details['price']
            total = quantity * price
            total_cost += total
            
            list_item = QListWidgetItem()
            item_widget = QWidget()
            item_layout = QHBoxLayout()

            item_label = QLabel(f"{item} x{quantity} - ${total}")
            item_label.setAlignment(Qt.AlignLeft)

            # - 버튼 추가
            minus_button = QPushButton("-")
            minus_button.setFixedSize(25, 25)
            minus_button.clicked.connect(lambda checked, i=item: self.update_item_quantity(i, self.selected_order[i]['quantity'] - 1))

            # + 버튼 추가
            plus_button = QPushButton("+")
            plus_button.setFixedSize(25, 25)
            plus_button.clicked.connect(lambda checked, i=item: self.update_item_quantity(i, self.selected_order[i]['quantity'] + 1))

            item_layout.addWidget(item_label)
            item_layout.addWidget(minus_button)
            item_layout.addWidget(plus_button)
            item_widget.setLayout(item_layout)

            list_item.setSizeHint(item_widget.sizeHint())
            self.order_list.addItem(list_item)
            self.order_list.setItemWidget(list_item, item_widget)

        self.total_price_label.setText(f"Total: ${total_cost}")

    def update_item_quantity(self, item, quantity):
        if quantity <= 0:
            self.selected_order.pop(item, None)  # 수량이 0 이하이면 삭제
        else:
            self.selected_order[item]['quantity'] = quantity  # 새로운 수량 설정
        self.update_order_list()

    def clear_order(self):
        self.selected_order.clear()
        self.update_order_list()
        self.total_price_label.setText("Total: $0.00")


    def say_gpt(self):
        # 팝업창 띄우기
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("음성 입력")
        msg_box.setText("말하시오... (확인 버튼을 누르면 녹음이 종료됩니다)")
        msg_box.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)

        # 녹음을 시작하도록 타이머 설정 (팝업이 뜬 후 실행)
        QTimer.singleShot(100, self.start_recording)  # 100ms 후 녹음 시작

        # 확인 버튼을 누르면 녹음 종료 후 처리
        if msg_box.exec_() == QMessageBox.Ok:
            print("녹음 종료 요청됨")
            self.audio_worker.requestInterruption()  # 안전한 종료 요청

    def start_recording(self):
        print("녹음 시작")
        self.audio_worker = RecordAudioWorker(silence_duration=1)  # 녹음 스레드 실행
        self.audio_worker.recording_done.connect(self.handle_audio_recording_done)
        self.audio_worker.start()


    def handle_audio_recording_done(self, frames):
        audio_file = "output.wav"
        save_audio(frames, audio_file)

        print("오디오 녹음 완료, GPT 분석 시작")
        # GPTWorker 실행
        self.gpt_worker = GPTWorker(audio_file, history_messages, client)
        self.gpt_worker.response_ready.connect(self.handle_gpt_response)
        self.gpt_worker.error_occurred.connect(self.handle_gpt_error)
        self.gpt_worker.start()

    def handle_gpt_response(self, response):
        print("AI의 응답:", response)
        
        # TTSWorker를 사용하여 TTS 백그라운드 실행
        self.tts_worker = TTSWorker(response)
        self.tts_worker.start()

        # UI에 응답 표시
        QMessageBox.information(self, "GPT 응답", response)

    def handle_gpt_error(self, error_message):
        QMessageBox.critical(self, "오류", error_message)

def main():
    rclpy.init()
    node = NODE()
    ros_thread = threading.Thread(target=lambda : rclpy.spin(node))
    ros_thread.start()
    app = QApplication(sys.argv)  # QApplication 생성
    kiosk = BurgerKiosk(node)  # 키오스크 창 생성
    kiosk.show()  # 창 표시
#----------------------------------------------------------------------
    try:
        sys.exit(app.exec_())  # 애플리케이션 실행
    except KeyboardInterrupt:
        passs

    finally:
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()  # ROS 스레드를 종료하도록 변경

if __name__ == '__main__':
    main()
