import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile
from PyQt5.QtWidgets import QApplication, QPushButton, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Bool

class TurtleBot3Navigation(Node):
    def __init__(self):
        super().__init__('turtlebot3_navigation')

        self.delivered = True   # ì™„ìˆ˜ í”Œëž˜ê·¸
        self.served = False # ë‹¨ì¼ ë°°ë‹¬ í ì™„ìˆ˜ í”Œëž˜ê·¸
        self.can_serve = False  # ì¸ì ‘ ë°°ë‹¬ ê°€ë¶€ í”Œëž˜ê·¸
        self.table_queue = []

        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )   # QoS 프로필 설정하여 발행된 토픽을 누락 없이 구독

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

        self.ready_publisher_ = self.create_publisher(Bool, 'ready_to_go', 10)
        self.create_subscription(Int32MultiArray, 'table_topic', self.run_callback, qos_profile)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # GUI êµ¬ì„±
        self.app = QApplication([])
        self.window = QWidget()
        self.push_button = QPushButton('수령', self.window)
        self.push_button.setEnabled(False)
        self.push_button.clicked.connect(self.handle_push_button)
        self.message_label = QLabel('서비스 대기중 ...',self.window)
        self.message_label.setAlignment(Qt.AlignCenter)
        
        # GUI ë ˆì´ì•„ì›ƒ
        self.window.setLayout(QVBoxLayout())
        self.window.layout().addWidget(self.push_button)
        self.window.layout().addWidget(self.message_label)
        self.window.show()

    def create_pose_stamped(self, x, y, z=0.0):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

    def run_callback(self, msg):
        if self.delivered:
            self.cross_num=5    # ë°°ë‹¬ ì¶œë°œ ì „ êµì°¨ë¡œ ë²ˆí˜¸ 5ë¡œ ì´ˆê¸°í™”
            self.table_queue = msg.data
            if self.table_queue:    # ë°°ë‹¬ íê°€ ìžˆìœ¼ë©´ ì™„ìˆ˜ í”Œëž˜ê·¸ ë‚´ë¦¬ê³  ì¶œë°œ
                self.delivered = False
                self.execute_navigation(self.table_queue[0])
            else:
                self.update_message('서비스 대기 중입니다.')

    def handle_push_button(self):   # ìˆ˜ë ¹ ë²„íŠ¼ í´ë¦­
        if self.table_queue:
            del self.table_queue[0] # ë°°ë‹¬ í ë§¨ ì•žìžë¦¬ ì‚­ì œ
            if self.table_queue:
                self.served = False
                self.can_serve = False
                self.push_button.setEnabled(False)  # ë°°ë‹¬ ì§€í‘œë“¤ ì´ˆê¸°í™”
                next_goal = self.table_queue[0]     # ë°°ë‹¬ í ì—…ë°ì´íŠ¸
                self.execute_navigation(next_goal)  # ì¶œë°œ
            else:
                self.delivered = True    # ì™„ìˆ˜ í”Œëž˜ê·¸ ì˜¬ë¦¬ê³  ëŒ€ê¸°
                self.update_message('서비스 대기 중입니다.')
                self.push_button.setEnabled(False)
        else:
            self.update_message('서비스 대기 중입니다.')    # ë°°ë‹¬ í ì—†ì„ ë•Œ ëˆ„ë¥¸ ê²½ìš° (line: , ë¹„í™œì„±í™” ìƒíƒœì§€ë§Œ)

    def execute_navigation(self, goal):
                      
        self.goal_point=goal
        self.goal_msg = NavigateToPose.Goal()
        self.serve_junction=self.cross_num*10+self.goal_point    # êµì°¨ë¡œ - í…Œì´ë¸” ì„œë¹™ ì ‘ì 
        
        ''' 
        # ì£¼ì„ì²˜ë¦¬: ì—†ëŠ” í…Œì´ë¸” ë²ˆí˜¸ ë°›ì„ ì¼ ì—†ìŒ & ëª¨ë“  ì¶œë°œ ì „ ì´ë¯¸ í…Œì´ë¸” í ì²´í¬í•¨
        if self.goal_point not in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]:
            self.update_message(f"ì—†ëŠ” í…Œì´ë¸”ìž…ë‹ˆë‹¤. : {self.goal_point}")
            return
        
        if not self.table_queue:
            self.update_message("ì„œë¹„ìŠ¤ ëŒ€ê¸° ì¤‘ìž…ë‹ˆë‹¤.")
            self.delivered = True
            return
        '''
                
        if self.goal_point != 0:
            self.update_message(f"{self.goal_point}번 테이블로 이동 중...")
        elif self.goal_point == 0:
            self.update_message("주방으로 복귀 중...")
        
        # goal_point í…Œì´ë¸” ë²ˆí˜¸ë¡œ ì´ë™ ëª©í‘œ ì„¤ì •
        if not self.served:
            if self.serve_junction in self.goal_position:
                self.can_serve = True
                self.goal_msg.pose = self.goal_position[self.serve_junction]
            else:        
                if self.goal_point in [1, 2, 4, 5]:
                    self.goal_msg.pose = self.goal_position[1]   # ëª©í‘œ êµì°¨ë¡œ ë¨¼ì € ì´ë™
                    self.cross_num=1 #êµì°¨ë¡œ ë²ˆí˜¸ ì—…ë°ì´íŠ¸
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
        node=TurtleBot3Navigation()
        if result.status == 4:            
            self.served == True
            if self.can_serve:  # ì¸ì ‘ ë°°ë‹¬ì´ì—ˆë‹¤ë©´ ë°°ë‹¬ ì™„ë£Œ
                self.update_message(f"{self.goal_point}번 테이블, 주문하신 음식입니다.")
                self.push_button.setEnabled(True)
            elif self.goal_point != 0:  # ì¸ì ‘ ë°°ë‹¬ / ë³µê·€ê°€ ì•„ë‹ˆì—ˆë‹¤ë©´ ì¸ì ‘ ë°°ë‹¬ ì§„í–‰
                self.served = False # ì•„ì§ ë°°ë‹¬ëœ ê±° ì•„ë‹˜
                self.execute_navigation(self.goal_point)
            else:   # ë³µê·€
                self.served = False
                self.can_serve = False
                self.push_button.setEnabled(False)  # ë°°ë‹¬ ì§€í‘œ ì´ˆê¸°í™”í•˜ê³  ëŒ€ê¸°
                self.update_message("서비스 대기 중입니다.")
                node.publish_message(True)  # 도착했다는 토픽 발행
            
        elif result.status == 2:  # CANCELED ìƒíƒœ
            self.update_message("목표가 취소되었습니다.")
        elif result.status == 3:  # ABORTED ìƒíƒœ
            self.update_message("목표에 도달하지 못했습니다.")
        else:
            self.update_message("알 수 없는 상태입니다.")

    def publish_message(self, data: bool):
        msg = Bool()
        msg.data = data
        self.ready_publisher_.publish(msg)
            
    def update_message(self, message):  # GUI ìŠ¤ë ˆë“œì—ì„œ ë©”ì‹œì§€ ì—…ë°ì´íŠ¸í•˜ê¸°
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
