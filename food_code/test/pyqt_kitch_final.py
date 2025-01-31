import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QGridLayout, QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import pyqtSignal, QObject
import threading
from pydub import AudioSegment
from pydub.playback import play
import rclpy
from rclpy.node import Node
from food_msg.msg import Ordermsg  # 구독할 메시지 타입
from food_msg.srv import Datawait  # 상황 모니터링 여부


def play_sound_segment(file_path, start_time, end_time):
    """
    오디오 파일의 특정 구간만 재생
    :param file_path: 오디오 파일 경로
    :param start_time: 시작 시간(초)
    :param end_time: 종료 시간(초)
    """
    try:
        # 오디오 파일 로드
        audio = AudioSegment.from_file(file_path)

        # 시간 범위 설정 (밀리초 단위로 변환)
        start_ms = start_time * 1000
        end_ms = end_time * 1000

        # 지정된 구간 추출
        segment = audio[start_ms:end_ms]

        # 소리 재생 (별도 스레드에서 실행)
        threading.Thread(target=play, args=(segment,)).start()
    except Exception as e:
        print(f"Failed to play sound segment: {e}")


class KitchenGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node  # ROS2 노드
        self.initUI()
        self.selected_tables = []  # 선택된 테이블 목록을 저장하는 리스트

    def initUI(self):
        self.setWindowTitle('주방 GUI')  # 윈도우 제목 설정
        self.setGeometry(100, 100, 400, 400)  # 윈도우 크기 및 위치 설정

        # Main layout
        main_layout = QVBoxLayout()  # 전체 레이아웃은 수직 박스 레이아웃

        # Table buttons layout
        table_layout = QGridLayout()  # 테이블 버튼들을 위한 그리드 레이아웃

        # Create 9 table buttons
        self.table_buttons = {}  # 각 테이블 버튼 객체를 저장할 딕셔너리
        self.table_status = {i: None for i in range(1, 10)}  # 테이블 상태를 저장하는 딕셔너리 (초기 상태는 None)

        for i in range(1, 10):
            btn = QPushButton(f'{i}')  # 버튼에 테이블 번호를 텍스트로 설정
            btn.setFixedSize(300, 300)  # 버튼 크기 설정
            btn.setCheckable(True)  # 버튼을 토글 가능하게 설정

            # 버튼 스타일 설정
            btn.setStyleSheet("""
                QPushButton {
                    text-align: left top;  /* 텍스트를 왼쪽 상단에 정렬 */
                    padding: 5px;  /* 여백 설정 */
                    font-size: 30px;  /* 버튼 글자 크기 설정 */
                }
                QPushButton:checked {
                    background-color: yellow;  /* 선택된 상태의 버튼 색상 */
                }
            """)

            # 버튼 클릭 이벤트 연결
            btn.clicked.connect(lambda checked, table_num=i: self.handle_button_click(table_num))

            self.table_buttons[i] = btn  # 딕셔너리에 버튼 추가
            table_layout.addWidget(btn, (i - 1) // 3, (i - 1) % 3)  # 그리드 레이아웃에 버튼 배치

        # Add table layout to main layout
        main_layout.addLayout(table_layout)  # 테이블 버튼 레이아웃 추가

        # Label to show table status
        self.status_label = QLabel('Table Status: Listening for updates...')  # 상태 레이블 초기 텍스트 설정
        self.status_label.setStyleSheet("font-size: 16px;")  # 상태 레이블 스타일 설정
        main_layout.addWidget(self.status_label)  # 상태 레이블 추가

        # Send button layout
        send_layout = QHBoxLayout()  # 수평 레이아웃 생성
        self.send_button = QPushButton('Send')  # Send 버튼 생성
        self.send_button.clicked.connect(self.send_message)  # Send 버튼 클릭 이벤트 연결
        send_layout.addStretch()  # 레이아웃 우측 정렬을 위한 빈 공간 추가
        send_layout.addWidget(self.send_button)  # Send 버튼 추가

        # Add send button layout to main layout
        main_layout.addLayout(send_layout)  # Send 버튼 레이아웃 추가

        # Add main layout
        self.setLayout(main_layout)  # 전체 레이아웃 설정

    def handle_button_click(self, table_num):
        """테이블 버튼 클릭 처리"""
        if table_num in self.selected_tables:
            self.selected_tables.remove(table_num)  # 선택 해제 시 목록에서 제거
        else:
            self.selected_tables.append(table_num)  # 선택 시 목록에 추가

        # 상태 레이블 업데이트
        selected_text = ', '.join(map(str, self.selected_tables))  # 선택된 테이블 번호를 문자열로 변환
        self.status_label.setText(f'Table Status: Selected Tables - {selected_text}')  # 상태 레이블 텍스트 업데이트

    def send_message(self):
        """Send 버튼 클릭 시 처리"""
        if self.node is not None:
            for table_num in self.selected_tables:
                # 로그 출력
                self.node.get_logger().info(f'Clearing Table {table_num}')
                
                # 테이블 상태 초기화
                self.table_status[table_num] = None

                # 버튼 초기화
                self.table_buttons[table_num].setChecked(False)  # 버튼 선택 상태 해제
                self.table_buttons[table_num].setText(f'{table_num}')  # 버튼 텍스트 초기화

            # 선택된 테이블 목록 초기화
            self.selected_tables.clear()

            # 상태 레이블 업데이트
            self.status_label.setText('Table Status: All selections cleared!')


    def update_table_status(self, table_num, food_items):
        """테이블 상태 업데이트"""
        self.table_status[table_num] = food_items  # 테이블 상태 저장

        # 버튼 텍스트 업데이트
        food_display = "\n".join([f"{name} x {count}" for name, count in food_items])  # 음식 목록을 문자열로 구성
        self.table_buttons[table_num].setText(f"{table_num}\n{food_display}")  # 버튼에 음식 목록 표시


    def check_table_status(self, table_num):
        """테이블 상태를 확인 (서비스를 위한 메소드)"""
        return self.table_status[table_num] is None


class KitchenSubscriber(Node):
    def __init__(self, gui):
        super().__init__('kitchen_subscriber')  # ROS2 노드 이름 설정
        self.gui = gui  # GUI 객체 저장
        self.service = self.create_service(Datawait, 'check_request', self.handle_check_request)
        self.get_logger().info('Kitchen Service Ready')

        # Subscriber 생성
        self.subscription = self.create_subscription(
            Ordermsg,  # 구독할 메시지 타입
            'order_topic',  # 구독할 토픽 이름
            self.listener_callback,  # 콜백 함수
            10  # 큐 크기 설정
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Kitchen Subscriber Ready')  # 초기화 로그 출력

    def listener_callback(self, msg):
        """토픽 메시지 수신 시 처리"""
        self.get_logger().info(
            f"Received Order: Table {msg.table_num}, Food ID: {msg.food_id}, Count: {msg.food_count}"
        )  # 수신 메시지 로그 출력

                # 소리 재생 (1초에서 5초 구간)
        try:
            play_sound_segment('/home/happy/Desktop/song.mp3', start_time=1, end_time=2)
        except Exception as e:
            self.get_logger().error(f"Failed to play sound segment: {e}")

        table_num = msg.table_num  # 메시지에서 테이블 번호 추출
        food_id_list = msg.food_id  # 메시지에서 음식 ID 리스트 추출
        food_count_list = msg.food_count  # 메시지에서 음식 수량 리스트 추출

        # food_count가 array 타입일 경우 리스트로 변환
        if not isinstance(food_count_list, list):
            try:
                food_count_list = list(food_count_list)  # array를 리스트로 변환
            except TypeError as e:
                self.get_logger().error(f"Failed to convert food_count to list: {e}")
                return

        # 음식 ID와 수량 리스트 길이 검증
        if len(food_id_list) != len(food_count_list):
            self.get_logger().error(
                f"Mismatch between food_id and food_count lengths: {len(food_id_list)} vs {len(food_count_list)}"
            )
            return

        # 음식 정보 구성
        food_items = [(food_id_list[i], food_count_list[i]) for i in range(len(food_id_list))]

        # GUI 업데이트
        self.gui.update_table_status(table_num, food_items)  # 음식 목록 전달


    def handle_check_request(self, request, response):
        table_num = request.table_num_1

        # GUI에서 테이블 상태 확인
        if self.gui.check_table_status(table_num):
            response.success_1 = 1  # 빈 테이블
            self.get_logger().info(f"Table {table_num} is empty.")
        else:
            response.success_1 = 0  # 비어 있지 않음
            self.get_logger().info(f"Table {table_num} is occupied.")

        return response


def main(args=None):
    rclpy.init(args=args)  # ROS2 초기화

    app = QApplication(sys.argv)  # PyQt 애플리케이션 생성

    # GUI와 ROS2 구독자 노드 연결
    gui = KitchenGUI(None)  # GUI 생성
    subscriber_node = KitchenSubscriber(gui)  # ROS2 구독자 노드 생성
    gui.node = subscriber_node  # GUI에 노드 연결

    # ROS2 스레드 실행
    ros_thread = threading.Thread(target=rclpy.spin, args=(subscriber_node,))  # ROS2 스레드 생성 및 실행
    ros_thread.start()

    # GUI 실행
    gui.show()  # GUI 표시
    app.exec_()  # PyQt 이벤트 루프 실행

    # 종료
    subscriber_node.destroy_node()  # 노드 파괴
    rclpy.shutdown()  # ROS2 종료
    ros_thread.join()  # 스레드 종료 대기


if __name__ == '__main__':
    main()  # 메인 함수 실행