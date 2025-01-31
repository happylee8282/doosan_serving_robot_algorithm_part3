import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile
from PyQt5.QtWidgets import QApplication, QPushButton, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt
from food_msg.srv import Navigationok  # 상황 모니터링 여부

class TurtleBot3Navigation(Node):
    def __init__(self):
        super().__init__('turtlebot3_navigation')

        self.delivered = True   # 모든 배달 완료 여부
        self.served = False # 단일 테이블 배달 완료 여부
        self.can_serve = False  # 목적지에서 서비스 제공 가능 여부
        self.table_queue = []  # 배달할 테이블 목록

        self.goal_position = {
            1: self.create_pose_stamped(
                                        x=1.48,
                                        y=1.04
            ),
            2: self.create_pose_stamped(
                                        x=1.5,
                                        y=-0.00462
            ),
            3: self.create_pose_stamped(
                                        x=2.59,
                                        y=1.08
            ),
            4: self.create_pose_stamped(
                                        x=2.61,
                                        y=-0.0247
            ),
            0: self.create_pose_stamped(
                                        x=-0.0331,
                                        y=-0.012
            ),
            11: self.create_pose_stamped(
                                        x=1.15,
                                        y=1.35
            ),
            12: self.create_pose_stamped(
                                        x=1.2,
                                        y=0.737
            ),
            14: self.create_pose_stamped(
                                        x=1.79,
                                        y=1.34
            ),
            15: self.create_pose_stamped(
                                        x=1.86,
                                        y=0.782
            ),
            22: self.create_pose_stamped(
                                        x=1.13,
                                        y=0.265
            ),
            23: self.create_pose_stamped(
                                        x=1.18,
                                        y=-0.371
            ),
            25: self.create_pose_stamped(
                                        x=1.86,
                                        y=0.239
            ),
            26: self.create_pose_stamped(
                                        x=1.87,
                                        y=-0.335
            ),
            34: self.create_pose_stamped(
                                        x=2.26,
                                        y=1.39
            ),
            35: self.create_pose_stamped(
                                        x=2.27,
                                        y=0.794
            ),
            45: self.create_pose_stamped(
                                        x=2.25,
                                        y=0.29
            ),
            46: self.create_pose_stamped(
                                        x=2.28,
                                        y=-0.368
            ),
            37: self.create_pose_stamped(
                                        x=2.92,
                                        y=1.37
            ),
            38: self.create_pose_stamped(
                                        x=2.91,
                                        y=0.771
            ),
            48: self.create_pose_stamped(
                                        x=2.91,
                                        y=0.251
            ),
            49: self.create_pose_stamped(
                                        x=2.93,
                                        y=-0.354
            )
        }

        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(Int32MultiArray, 'table_topic', self.run_callback, QoSProfile(depth=10))
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # GUI 구성
        self.app = QApplication([])
        self.window = QWidget()
        self.push_button = QPushButton('수령', self.window)
        self.push_button.setEnabled(False)
        self.push_button.clicked.connect(self.handle_push_button)
        self.message_label = QLabel('서비스 대기중 ...',self.window)
        self.message_label.setAlignment(Qt.AlignCenter)
        
        # GUI 레이아웃 설정        
        self.window.setLayout(QVBoxLayout())
        self.window.layout().addWidget(self.push_button)
        self.window.layout().addWidget(self.message_label)
        self.window.show()

    def create_pose_stamped(self, x, y, z=0.0):
        """ 특정 좌표에 대한 PoseStamped 메시지를 생성 """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

    def publish_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        initial_pose.pose.pose.position.x = -0.0188
        initial_pose.pose.pose.position.y = -0.0156
        initial_pose.pose.pose.position.z = -0.00143
        initial_pose.pose.pose.orientation = Quaternion(
            x=4.934622001685005e-05,
            y=0.002868069202798296,
            z=4.658814127285208e-06,
            w=0.9999958858526838
        )
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0685, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0685, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
        self.initialpose_publisher.publish(initial_pose)

    def run_callback(self, msg):
        """ 새로운 배달 요청을 수신했을 때 실행되는 콜백 """
        if self.delivered:
            self.publish_initial_pose()
            self.cross_num=5    
            self.table_queue = msg.data
            self.get_logger().info(f"새로운 배달 요청 수신: {self.table_queue}")
            if self.table_queue:   
                self.delivered = False
                self.execute_navigation(self.table_queue[0])
            else:
                self.update_message('서비스 대기 중입니다.')

    def handle_push_button(self):   # 수령 버튼 클릭
        self.get_logger().info("사용자가 '수령' 버튼을 클릭했습니다.")
        if self.table_queue:
            del self.table_queue[0] # 배달 완료된 테이블 제거
            if self.table_queue:
                self.served = False
                self.can_serve = False
                self.push_button.setEnabled(False)  
                next_goal = self.table_queue[0]     
                self.execute_navigation(next_goal) 
            else:
                self.delivered = True 
                self.update_message('서비스 대기 중입니다.')
                self.push_button.setEnabled(False)
        else:
            self.update_message('서비스 대기 중입니다.')    # 배달 큐 없을 때 누른 경우 (비활성화 상태지만)

    def execute_navigation(self, goal):
                      
        self.goal_point=goal
        self.get_logger().info(f"로봇이 {self.goal_point}번 테이블로 이동 시작.")
        self.goal_msg = NavigateToPose.Goal()
        self.serve_junction=self.cross_num*10+self.goal_point    # 교차로 테이블 접점
                    
        if self.goal_point != 0:
            self.update_message(f"{self.goal_point}번 테이블로 이동 중...")
        elif self.goal_point == 0:
            self.update_message("주방으로 복귀 중...")
        
        # goal_point 테이블 번호로 이동 목표 설정
        if not self.served:
            if self.serve_junction in self.goal_position:
                self.can_serve = True
                self.goal_msg.pose = self.goal_position[self.serve_junction]
            else:        
                if self.goal_point in [1, 2, 4, 5]:
                    self.goal_msg.pose = self.goal_position[1]   # 목표 교차로 먼저 이동
                    self.cross_num=1 # 교차로 번호 업데이트
                elif self.goal_point in [3, 6]:
                    self.goal_msg.pose = self.goal_position[2]
                    self.cross_num=2
                elif self.goal_point in [7, 8]:
                    self.goal_msg.pose = self.goal_position[3]
                    self.cross_num=3
                elif self.goal_point == 9:
                    self.goal_msg.pose = self.goal_position[4]
                    self.cross_num=4
                elif self.goal_point == 0:
                    self.goal_msg.pose = self.goal_position[0]
                    self.cross_num=5
                    self.delivered = True
                        
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(
            self.goal_msg,
            feedback_callback=self.nav_to_pose_client_feedback)
        self.send_goal_future.add_done_callback(self.nav_to_pose_client_action_goal)
        
        return True
    
    
    def nav_to_pose_client_feedback(self, feedback_msg):
        action_feedback = feedback_msg.feedback
        self.get_logger().info("Action feedback: {0}".format(action_feedback))
        
    def nav_to_pose_client_action_goal(self, future):
        goal_handle = future.result()
        self.action_result_future = goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.nav_to_pose_client_result)

    def nav_to_pose_client_result(self, future):
        result = future.result()
        if result.status == 4:
            self.served == True
            if self.can_serve:  # 인접 배달이었다면 배달 완료
                self.update_message(f"{self.goal_point}번 테이블, 주문하신 음식입니다.")
                self.get_logger().info(f"{self.goal_point}번 테이블 도착. 배달 완료.")
                self.push_button.setEnabled(True)
            elif self.goal_point != 0:  # 인접 배달 / 복귀가 아니었다면 인접 배달 진행
                self.served = False # 아직 배달되지 않음
                self.execute_navigation(self.goal_point)
            else:   # 복귀
                self.served = False
                self.can_serve = False
                self.push_button.setEnabled(False)  # 배달 지표 모두 초기화하고 대기
                self.update_message("서비스 대기 중입니다.")
            
        elif result.status == 2:  # CANCELED 상태
            self.update_message("목표가 취소되었습니다.")
        elif result.status == 3:  # ABORTED 상태
            self.update_message("목표에 도달하지 못했습니다.")
            self.get_logger().warn("Failed to find target point")
        else:
            self.update_message("알 수 없는 상태입니다.")
            self.get_logger().error("Unknown condition, please restart.")
            self.execute_navigation(self.table_queue[0])
            #logger_error
            #네비게이션이  error를 표시함으로써 service상에서 문제가 있다는 것을 발견 

            
    def update_message(self, message): 
        """ GUI의 메시지를 업데이트하고 로그를 기록 """
        self.message_label.setText(message)
        
    def set_emit_signal(self, emit_func):
        self.emit_signal = emit_func

    
def main(args=None):
    rclpy.init(args=args)
    navigation_node = TurtleBot3Navigation()

    ros_thread = threading.Thread(target=rclpy.spin, args=(navigation_node,))
    ros_thread.start()

    try:
        navigation_node.app.exec_()
    finally:
        navigation_node.destroy_node()
        ros_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
