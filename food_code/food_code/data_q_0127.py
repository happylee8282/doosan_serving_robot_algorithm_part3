import rclpy
from rclpy.node import Node
from food_msg.srv import Foodcustomer #손님이 보내는 srv

from food_msg.srv import Datawait #상황모니터있는여부 srv

from food_msg.msg import Ordermsg #food id랑 테이블 번호랑 음식 카운트 보내는것 msg
from array import array

import sqlite3
from datetime import datetime
from matplotlib import pyplot as plt
import numpy as np
import sys

import rclpy 
from rclpy.node import Node
from food_msg.msg import Ordermsg
from PyQt5.QtWidgets import QApplication, QWidget
import threading

from PyQt5.QtWidgets import QApplication, QMainWindow, QCalendarWidget, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QDate, Qt
from PyQt5.QtGui import QTextCharFormat
import time


class OrderServer(Node):
    def __init__(self):
        super().__init__('order_server')
        #서비스 손님gui
        self.service = self.create_service(Foodcustomer, 'pay_response', self.process_order_callback)

        #클라이언트 주방gui
        self.client = self.create_client(Datawait, 'check_request')

        # 요청 전송을 위한 타이머 설정 (1초마다 요청 전송)
        self.timer = self.create_timer(1.0, self.send_request)

        self.get_logger().info('Order Service Ready')

        # 퍼블리셔 생성
        self.publisher = self.create_publisher(Ordermsg, 'order_topic', 10)

        self.get_logger().info('Order Service and Publisher Ready')

        # 클래스 변수 초기화
        self.request_data = None
        self.table_1_orders = []  # 1번 테이블의 주문 리스트
        self.table_2_orders = []  # 1번 테이블의 주문 리스트
        self.table_3_orders = []  # 1번 테이블의 주문 리스트
        self.table_4_orders = []  # 1번 테이블의 주문 리스트
        self.table_5_orders = []  # 1번 테이블의 주문 리스트
        self.table_6_orders = []  # 1번 테이블의 주문 리스트
        self.table_7_orders = []  # 1번 테이블의 주문 리스트
        self.table_8_orders = []  # 1번 테이블의 주문 리스트
        self.table_9_orders = []  # 1번 테이블의 주문 리스트

#---------------------------------------------------------------------------------
    # 서비스 손님 GUI 성공 여부
    def process_order_callback(self, request, response):
        # 요청 데이터 출력
        self.get_logger().info(f"Table Number: {request.table_num}")
        self.get_logger().info(f"Food IDs: {request.food_id}")
        self.get_logger().info(f"Food Counts: {request.food_count}")


        # 요청 데이터를 리스트에 추가
        order_data = {
            'table_num': request.table_num,
            'food_id': request.food_id,
            'food_count': request.food_count
        }

        # db에 연결
        conn = sqlite3.connect('/home/happy/Desktop/newfile/order.db')
        cursor = conn.cursor()

        # 현재 시간
        current_time = datetime.now()
        date = current_time.strftime('%Y_%m_%d')
        time = current_time.strftime('%H:%M:%S')
        
        food_counts_list = list(order_data['food_count'])  # 리스트로 변환

        # 주문 내역
        for i in range(len(order_data['food_id'])):
            id = order_data['food_id'][i]
            count = food_counts_list[i]

            # 데이터 입력
        
            # 오늘 매출표가 존재하지 않으면 생성. table 이름은 'odres2025_01_24' 형태로 뒤에 날짜가 붙음
            exi = 'create table if not exists orders'+date+'(time TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP, menu TEXT, count INTEGER)'
            order = 'insert into orders'+date+'(time, menu, count) values (?, ?, ?)'
        
        # table생성, 주문을 시간, 종류, 갯수 순으로 삽입
            cursor.execute(exi)
            cursor.execute(order, (time, id, count))

        # 실행
            conn.commit()
        # 연결 종료
        conn.close()

        if request.table_num == 1:
            self.table_1_orders.append(order_data)
            self.get_logger().info(f"Added to Table 1 Orders: {order_data}")

        elif request.table_num == 2:
            self.table_2_orders.append(order_data)
            self.get_logger().info(f"Added to Table 2 Orders: {order_data}")

        elif request.table_num == 3:
            self.table_3_orders.append(order_data)
            self.get_logger().info(f"Added to Table 3 Orders: {order_data}")

        elif request.table_num == 4:
            self.table_4_orders.append(order_data)
            self.get_logger().info(f"Added to Table 4 Orders: {order_data}")

        elif request.table_num == 5:
            self.table_5_orders.append(order_data)
            self.get_logger().info(f"Added to Table 5 Orders: {order_data}")

        elif request.table_num == 6:
            self.table_6_orders.append(order_data)
            self.get_logger().info(f"Added to Table 6 Orders: {order_data}")

        elif request.table_num == 7:
            self.table_7_orders.append(order_data)
            self.get_logger().info(f"Added to Table 7 Orders: {order_data}")

        elif request.table_num == 8:
            self.table_8_orders.append(order_data)
            self.get_logger().info(f"Added to Table 8 Orders: {order_data}")

        elif request.table_num == 9:
            self.table_9_orders.append(order_data)
            self.get_logger().info(f"Added to Table 9 Orders: {order_data}")

        # 현재는 성공 상태만 반환
        response.success = 1  # 1: 성공, 0: 실패
        return response

#---------------------------------------------------------------------------------
    #클라이언트 주방gui 성공했니?
    def send_request(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for kitchen service...')
            return  # 서비스가 준비되지 않으면 반환


        # 테이블 번호별로 요청 생성 및 전송
        for table_num in range(1, 10):  # 1번부터 9번 테이블까지
            request = Datawait.Request()
            request.table_num_1 = table_num  # 테이블 번호 설정
            self.future = self.client.call_async(request)
            self.future.add_done_callback(lambda future, table_num=table_num: self.handle_response(future, table_num))

#---------------------------------------------------------------------------------
    #서비스 주방gui 성공했다
    def handle_response(self, future, table_num):
        try:
            response = future.result()
            if response.success_1 == 1:
                self.get_logger().info("Kitchen is ready. Proceeding with order.")
                self.topic_part(table_num)  # 토픽 발행
            else:
                self.get_logger().info("Kitchen is not ready.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            #logger_error
            #서비스가 연결이 안되었을떄 error를 표시함으로써 service상에서 문제가 있다는 것을 발견 
#---------------------------------------------------------------------------------
            # 토픽 주방 데이터 보내기
    def topic_part(self, table_num):
        orders = getattr(self, f"table_{table_num}_orders", None)

        # 첫 번째 주문 가져오기
        first_order = orders.pop(0)

        # 메시지 생성
        msg = Ordermsg()
        msg.table_num = first_order['table_num']
        msg.food_id = first_order['food_id']  # 리스트 그대로 사용
        msg.food_count = list(first_order['food_count'])  # 배열을 리스트로 변환

        # 메시지 발행
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: Table {msg.table_num}, Food IDs: {msg.food_id}, Counts: {msg.food_count}")

def main(args=None):
    rclpy.init(args=args)
    node = OrderServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
