import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from delivery_interfaces.msg import MenuRecommend
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QFrame, QGridLayout, QScrollArea
from PyQt5.QtCore import Qt, QTimer
import sqlite3
import sys
from kiosk.dbfuncs import *
from functools import partial
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
import threading

class KitchenDisplayNode(Node):
    def __init__(self):
        super().__init__('kitchen_display')
        self.declare_parameter('table_num', 9)
        table_num = self.get_parameter('table_num').value
        self.QOS_RKL10T = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.callback_group = ReentrantCallbackGroup()

        self.subscription_list = []
        for i in range(1, table_num+1):
            self.subscription_list.append(
                self.create_subscription(String, f'/table_{i}/order_topic', partial(self.receive_order), self.QOS_RKL10T, callback_group=self.callback_group))
        self.orders = []
        self.init_database()

        # 로봇 제어 데이터
        self.delivery_publisher_ = self.create_publisher(String, 'delivery_request_topic', 10, callback_group=self.callback_group)

    def init_database(self):
        self.db_path = 'orders.db'
        try:
            load_db()
            calculate_profit()
            save_recommended_menu()
            self.get_logger().info("Database initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize database: {e}")

    def receive_order(self, msg):
        self.orders.append(msg.data)
        self.get_logger().info(f'Received: "{msg.data}"')

class KitchenGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        #로봇 배달 상태
        self.robot_state = '서빙로봇 대기중'
        #추천메뉴 서비스 서버
        # 추천 메뉴 수신
        self.node.recommended_sub = self.node.create_subscription(MenuRecommend, f'/table_1/menu_recommend', self.recommended_callback, self.node.QOS_RKL10T, callback_group=self.node.callback_group)
        #배달 상태 메세지 subscriber
        self.node.robot_status_sub = self.node.create_subscription(String, 'robot_status_topic', self.robot_status_callback, 10, callback_group=self.node.callback_group)

        #5초마다 주방 정보 출력
        self.node.timer = self.node.create_timer(5, self.show_sales_info)

        self.initUI()

    def initUI(self):
        self.setWindowTitle('주방 디스플레이')
        self.setGeometry(100, 100, 1200, 800)
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        button_layout = QGridLayout()
        sales_button = QPushButton("매출 보기")
        sales_button.clicked.connect(self.show_sales)
        button_layout.addWidget(sales_button, 0, 0)
        close_button = QPushButton("영업 종료")
        close_button.clicked.connect(self.end_of_day)
        button_layout.addWidget(close_button, 0, 1)
        self.main_layout.addLayout(button_layout)

        self.card_layout = QVBoxLayout()
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_content.setLayout(self.card_layout)
        self.scroll_area.setWidget(scroll_content)
        self.main_layout.addWidget(self.scroll_area)
        
        self.sales_display = QLabel("매출 정보가 여기에 표시됩니다.")
        self.sales_display.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.main_layout.addWidget(self.sales_display)

        self.robot_state_display = QLabel(self.robot_state)
        self.robot_state_display.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.main_layout.addWidget(self.robot_state_display)

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_cards)
        self.timer.start(1000)

    def refresh_cards(self):
        for i in reversed(range(self.card_layout.count())):
            widget = self.card_layout.takeAt(i).widget()
            if widget:
                widget.deleteLater()

        table_colors = {
            "Table_1": "#ADD8E6",
            "Table_2": "#FFB6C1",
            "Table_3": "#FFFFE0",
            "Table_4": "#98FB98",
            "Table_5": "#DDA0DD",
            "Table_6": "#FFA07A",
            "Table_7": "#FFC0CB",
            "Table_8": "#87CEFA",
            "Table_9": "#FFD700",
        }

        table_orders = {}
        for order in self.node.orders:
            parts = order.split(" - ")
            table_id = parts[0]
            details = " - ".join(parts[1:])
            if table_id not in table_orders:
                table_orders[table_id] = []
            table_orders[table_id].append(details)

        for table_id, orders in table_orders.items():
            frame = QFrame()
            frame.setFrameShape(QFrame.Box)
            frame.setStyleSheet(f"background-color: {table_colors.get(table_id, '#FFFFFF')};")
            layout = QVBoxLayout()

            title = QLabel(f"{table_id}")
            title.setStyleSheet("font-weight: bold; font-size: 16px;")
            layout.addWidget(title)

            for order in orders:
                order_label = QLabel(order)
                layout.addWidget(order_label)

            complete_button = QPushButton("완료")
            complete_button.clicked.connect(lambda checked, t=table_id: self.complete_orders(t))
            layout.addWidget(complete_button)

            frame.setLayout(layout)
            self.card_layout.addWidget(frame)

    def recommended_callback(self, msg):
        '''
        string[] menu       : 메뉴 이름
        int32[] isrecommend : 1 추천, -1 -순이익
        '''
        menu = msg.menu
        isrecommend = msg.isrecommend

        recommend_menu = menu[isrecommend.index(1)]
        self.node.get_logger().info(f"recommended = {recommend_menu}")
        update_table_info('orders', 'menu', recommend_menu, 'table_id', "추천")

    def robot_status_callback(self, status):
        self.robot_state = status.data
        self.update_robot_state()

    def update_robot_state(self):
        self.robot_state_display.setText(self.robot_state)

    def show_sales_info(self):
        """
        display
        총 매출
        매출 순이익
        재료별 재고 현황
        """
        try : 
            menu, price, quantity = zip(*get_table_info('orders', ['menu', 'price', 'quantity']))
        except ValueError: 
            price, quantity = 0, 0
        
        menu_list = get_table_info('menu', 'name')
        menu_list = [i[0] for i in menu_list]
        price = [price] if isinstance(price, int) else price
        quantity = [quantity] if isinstance(quantity, int) else quantity
        ingredient, stock_quantity, cost = zip(*get_table_info('ingredient', ['ingredient_name', 'stock_quantity', 'cost']))
        profit = get_table_info('menu', 'profit')
        profit = [i[0] for i in profit]

        try:
            # 매출 계산 로직
            revenue = sum([i * j for i, j in zip(price, quantity)])
            self.node.get_logger().info(f"💰 총 매출: {revenue:,} 원")
        except ValueError:
            self.node.get_logger().warn("No sales data available.")

        self.node.get_logger().info('\n' + '-' * 30)
        self.node.get_logger().info('📊 [주문 시스템 요약]')
        self.node.get_logger().info(f'💰 총 매출: {revenue:,} 원')

        self.node.get_logger().info('\n📜 메뉴 목록:')
        self.node.get_logger().info(f'  {menu_list}')

        self.node.get_logger().info('\n📈 메뉴별 순이익:')
        self.node.get_logger().info(f'  {profit}')

        self.node.get_logger().info('\n🛒 재료 목록:')
        self.node.get_logger().info(f'  {ingredient}')

        self.node.get_logger().info('\n🏷️ 재료 가격 (원):')
        self.node.get_logger().info(f'  {cost}')

        self.node.get_logger().info('\n📦 재료 재고 현황:')
        self.node.get_logger().info(f'  {stock_quantity}')

        self.node.get_logger().info('-' * 30 + '\n')

    def complete_orders(self, table_id):
        self.node.orders = [order for order in self.node.orders if not order.startswith(table_id)]

        # 테이블 ID를 퍼블리시
        msg = String()
        msg.data = table_id
        self.node.get_logger().info(f'Publishing delivery request for table_id: "{msg.data}"')
        self.node.delivery_publisher_.publish(msg)

        self.refresh_cards()

    def show_sales(self):
        db_path = self.node.db_path
        conn = sqlite3.connect(db_path)
        c = conn.cursor()
        c.execute("SELECT menu, SUM(price * quantity), SUM(quantity) FROM orders GROUP BY menu")
        sales_data = c.fetchall()
        conn.close()

        if not sales_data:
            self.sales_display.setText("매출 정보가 없습니다.")
            return

        total_sales = sum(data[1] for data in sales_data)
        max_sales_item = max(sales_data, key=lambda x: x[1])
        min_sales_item = min(sales_data, key=lambda x: x[1])

        self.sales_display.setText(
            f"총 판매 금액: {total_sales}원\n"
            f"최고 매출 메뉴: {max_sales_item[0]} ({max_sales_item[1]}원)\n"
            f"최저 매출 메뉴: {min_sales_item[0]} ({min_sales_item[1]}원)"
        )

    def end_of_day(self):
        load_db(['orders.db', 'ingredient.db'])
        self.refresh_cards()
        self.sales_display.setText("영업 종료되었습니다. 주문정보를 초기화합니다.")
    
    def end_of_day1(self):
        db_path = self.node.db_path
        conn = sqlite3.connect(db_path)
        c = conn.cursor()

        c.execute("""
            SELECT menu, SUM(price * quantity) as total_sales
            FROM orders
            GROUP BY menu
            ORDER BY total_sales ASC
            LIMIT 1
        """)
        min_sales_item = c.fetchone()
        c.execute("DELETE FROM orders")
        if min_sales_item:
            c.execute("""
                INSERT INTO orders (table_id, menu, price, quantity)
                VALUES (?, ?, ?, ?)
            """, ("추천", min_sales_item[0], 0, 0))
        conn.commit()
        conn.close()

        self.node.orders.clear()
        self.refresh_cards()
        self.sales_display.setText("영업 종료되었습니다. 최저 매출 메뉴는 저장되었습니다.")

def start_ros2_node(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = KitchenDisplayNode()

    app = QApplication(sys.argv)
    gui = KitchenGUI(node)
    gui.show()

    # ROS 2 노드를 별도의 스레드에서 실행
    ros2_thread = threading.Thread(target=start_ros2_node, args=(node,))
    ros2_thread.start()

    # GUI 실행
    sys.exit(app.exec_())

    node.destroy_node()
    rclpy.shutdown()
    ros2_thread.join()

if __name__ == "__main__":
    main()