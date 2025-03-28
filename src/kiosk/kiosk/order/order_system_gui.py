import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from delivery_interfaces.srv import CancelMenu, AddMenu
from delivery_interfaces.msg import MenuRecommend
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QListWidget, QMessageBox, QGridLayout, QFrame
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap
import sqlite3
import os
import sys
from kiosk.dbfuncs import *
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

class OrderSystemNode(Node):
    def __init__(self, table_id, namespace='table_1'):
        super().__init__('order_system', namespace=namespace)
        self.QOS_RKL10T = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        
        # 주문 완료 토픽
        self.publisher_ = self.create_publisher(String, 'order_topic', self.QOS_RKL10T)

        # 주문버튼 클릭 시 재료소진여부 확인
        self.order_client = self.create_client(AddMenu, 'check_stock', qos_profile=self.QOS_RKL10V)
        # 주문 취소시 재료 현황 복구
        self.cancel_client = self.create_client(CancelMenu, 'menu_cancel', qos_profile=self.QOS_RKL10V)

        self.declare_parameter('table_id', table_id)
        table_id = self.get_parameter('table_id').value
        self.table_id = table_id

        while not self.order_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The order_client service not available.')
        while not self.cancel_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The cancle_client service not available.')

    def recommanded_callback(self, msg):
        '''
        string[] menu
        int32[] isrecommnend
        '''
        self.get_logger().info("menu_recommnend received")
        menu = msg.menu
        isrecommend = msg.isrecommend
        recommended_menu = menu[isrecommend.index(1)]
        name, price, category = zip(*get_table_info('menu', ['name', 'price', 'category'], args=f'WHERE name =="{recommended_menu}"'))

        try : table_id, m, p, quantity = get_table_info('orders', ['table_id', 'menu', 'price', 'quantity'], args=f'WHERE menu == "{name}"')
        except : table_id, m, p, quantity = '추천', name, 0, 0
        finally : 
            table_id = str(table_id) if table_id is not None else '추천'
            m = str(m)
            p = int(p) if p is not None else 0
            quantity = int(quantity) if quantity is not None else 0

            update_table_info(
                table_name='orders',
                updated_column=['table_id', 'menu', 'price', 'quantity'],
                values=[str(table_id), str(m), int(p), int(quantity)],
                where_column='table_id',
                where_value='추천'
            )

        return name, price, category
            

class OrderSystemGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.pkg_name = 'kiosk'
        self.pkg_path = os.path.join(get_package_share_directory(self.pkg_name))
        self.node = node
        self.cart = []
        self.current_category = "추천"

        # 추천 메뉴 수신
        self.node.recommended_sub = self.node.create_subscription(MenuRecommend, 'menu_recommend', self.recommanded_callback, self.node.QOS_RKL10T)
        self.initUI()

    def initUI(self):
        # db 세팅
        load_db()
        calculate_profit()
        save_recommended_menu()

        self.recommended_menu = self.load_recommended_menu()  # 추천 메뉴 로드
        self.setWindowTitle(f"테이블 {self.node.table_id} 주문 시스템")
        self.setGeometry(100, 100, 1200, 800)

        # 전체 창 스타일 적용
        self.setStyleSheet("""
            background-color: #F5F5F5;
            font-family: Arial, sans-serif;
        """)

        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        # 카테고리 버튼
        category_layout = QHBoxLayout()

        button_style = """
            QPushButton {
                background-color: #FFCC00;
                border: 1px solid #E0E0E0;
                border-radius: 10px;
                padding: 10px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #FFD633;
            }
            QPushButton:pressed {
                background-color: #E6B800;
            }
        """

        recommend_button = QPushButton("오늘의 추천 메뉴")
        recommend_button.setStyleSheet(button_style)
        recommend_button.clicked.connect(lambda: self.update_menu("추천"))
        category_layout.addWidget(recommend_button)

        drink_button = QPushButton("음료")
        drink_button.setStyleSheet(button_style)
        drink_button.clicked.connect(lambda: self.update_menu("음료"))
        category_layout.addWidget(drink_button)

        dessert_button = QPushButton("디저트")
        dessert_button.setStyleSheet(button_style)
        dessert_button.clicked.connect(lambda: self.update_menu("디저트"))
        category_layout.addWidget(dessert_button)

        shake_button = QPushButton("쉐이크")
        shake_button.setStyleSheet(button_style)
        shake_button.clicked.connect(lambda: self.update_menu("쉐이크"))
        category_layout.addWidget(shake_button)

        self.main_layout.addLayout(category_layout)

        # 메뉴 UI
        self.menu_layout = QGridLayout()
        self.menu_layout.setContentsMargins(10, 10, 10, 10)
        self.menu_layout.setSpacing(15)
        self.menu = get_table_info('menu', ['name', 'price', 'category'])
        self.display_menu()

        menu_frame = QFrame()
        menu_frame.setLayout(self.menu_layout)
        menu_frame.setStyleSheet("background-color: #FFFAF0; border-radius: 10px; padding: 10px;")
        self.main_layout.addWidget(menu_frame)

        # 장바구니 UI 추가
        cart_layout = QVBoxLayout()

        cart_label = QLabel("장바구니")
        cart_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #333;")
        cart_layout.addWidget(cart_label)

        self.cart_list = QListWidget()
        self.cart_list.setStyleSheet("""
            background-color: #FFFFFF;
            border: 1px solid #CCCCCC;
            border-radius: 5px;
            padding: 5px;
        """)
        cart_layout.addWidget(self.cart_list)

        remove_button = QPushButton("선택 항목 제거")
        remove_button.setStyleSheet("""
            background-color: #FF4500;
            color: white;
            border-radius: 10px;
            padding: 10px;
        """)
        remove_button.clicked.connect(self.remove_from_cart)
        cart_layout.addWidget(remove_button)

        order_button = QPushButton("장바구니 주문하기")
        order_button.setStyleSheet("""
            background-color: #28A745;
            color: white;
            border-radius: 10px;
            padding: 10px;
            font-size: 16px;
        """)
        order_button.clicked.connect(self.submit_cart)
        cart_layout.addWidget(order_button)

        cart_frame = QFrame()
        cart_frame.setLayout(cart_layout)
        cart_frame.setStyleSheet("background-color: #E0F7FA; border-radius: 10px; padding: 10px;")
        self.main_layout.addWidget(cart_frame)

    def get_img_path(self, name:str, extension:str = '.jpeg'):
        image_dir = os.path.join(self.pkg_path, 'images', name)
        return image_dir + extension

    def recommanded_callback(self, msg):
        '''
        string[] menu
        int32[] isrecommnend
        '''
        self.node.get_logger().info("menu_recommnend received")
        menu = msg.menu
        isrecommend = msg.isrecommend
        recommended_menu = menu[isrecommend.index(1)]
        name, price, category = zip(*get_table_info('menu', ['name', 'price', 'category'], args=f'WHERE name =="{recommended_menu}"'))

        try : table_id, m, p, quantity = get_table_info('orders', ['table_id', 'menu', 'price', 'quantity'], args=f'WHERE menu == "{name}"')
        except : table_id, m, p, quantity = '추천', name, 0, 0
        finally : update_table_info('orders', ['table_id', 'menu', 'price', 'quantity'], [table_id, m, p, quantity], 'table_id', '추천')

        self.display_menu()

        #return name, price, category
    
    def load_recommended_menu(self):
        """전날 영업 종료 시 저장된 최저 매출 메뉴를 불러옵니다."""
        orders_db_path = "orders.db"
        menu_db_path = "menu.db"
        conn_orders = sqlite3.connect(orders_db_path)
        conn_menu = sqlite3.connect(menu_db_path)

        try:
            # 추천 메뉴 이름 가져오기
            c_orders = conn_orders.cursor()
            c_orders.execute("SELECT menu FROM orders WHERE table_id = '추천' LIMIT 1")
            result = c_orders.fetchone()
            if not result:
                print("추천 메뉴가 없습니다.")
                return None  # 추천 메뉴가 없을 경우

            recommended_menu_name = result[0]
            print(recommended_menu_name)

            # 메뉴 정보 가져오기
            menu_info = get_table_info('menu', ['name', 'price', 'category'], 'menu.db', f"WHERE name = '{recommended_menu_name}'")
            menu_info = menu_info[0]

            if menu_info:
                print(f"추천 메뉴 정보: {menu_info}")
                return menu_info  # (name, price, category)
            else:
                print("추천 메뉴가 menu 테이블에 없습니다.")
                return None
        except sqlite3.Error as e:
            print(f"Database error: {e}")
            return None
        finally:
            conn_orders.close()
            conn_menu.close()

    def display_menu(self):
        """현재 선택된 카테고리에 따라 메뉴를 표시."""
        for i in reversed(range(self.menu_layout.count())):
            widget = self.menu_layout.takeAt(i).widget()
            if widget:
                widget.deleteLater()

        if self.current_category == "추천":
            recommended_menu = self.load_recommended_menu()
            if not recommended_menu:
                # 추천 메뉴가 없으면 메시지 표시
                label = QLabel("추천 메뉴가 없습니다.")
                self.menu_layout.addWidget(label, 0, 0)
                return

            # 추천 메뉴 표시
            name, price, category = recommended_menu
            image_path = self.get_img_path(name)

            frame = QFrame(self)
            frame.setFrameShape(QFrame.Box)
            frame.setStyleSheet("background-color: #FFF8DC; border-radius: 10px; padding: 10px;")

            menu_layout_vbox = QVBoxLayout()

            if image_path and os.path.exists(image_path):
                pixmap = QPixmap(image_path).scaled(200, 200, Qt.KeepAspectRatio)
            else:
                pixmap = QPixmap()

            image_label = QLabel()
            image_label.setPixmap(pixmap)
            menu_layout_vbox.addWidget(image_label, alignment=Qt.AlignCenter)

            item_label = QLabel(f"{name} - {price}원")
            item_label.setAlignment(Qt.AlignCenter)
            menu_layout_vbox.addWidget(item_label)

            add_to_cart_button = QPushButton("장바구니 추가")
            add_to_cart_button.setStyleSheet("""
                background-color: #D3D3D3;
                border: 1px solid #A9A9A9;
                border-radius: 5px;
                padding: 5px;
                font-size: 12px;
                min-width: 120px;
            """)
            add_to_cart_button.clicked.connect(lambda checked, o=name, p=price: self.add_to_cart(o, p))
            menu_layout_vbox.addWidget(add_to_cart_button, alignment=Qt.AlignCenter)

            frame.setLayout(menu_layout_vbox)
            self.menu_layout.addWidget(frame, 0, 0)

            return

        # 다른 카테고리 메뉴 표시
        row, col = 0, 0
        for item, price, category in self.menu:
            if self.current_category != "전체" and category != self.current_category:
                continue
            image_path = self.get_img_path(item)


            frame = QFrame(self)
            frame.setFrameShape(QFrame.Box)
            frame.setStyleSheet("background-color: #FFF8DC; border-radius: 10px; padding: 10px;")

            menu_layout_vbox = QVBoxLayout()

            if image_path and os.path.exists(image_path):
                pixmap = QPixmap(image_path).scaled(200, 200, Qt.KeepAspectRatio)
            else:
                pixmap = QPixmap()

            image_label = QLabel()
            image_label.setPixmap(pixmap)
            menu_layout_vbox.addWidget(image_label, alignment=Qt.AlignCenter)

            item_label = QLabel(f"{item} - {price}원")
            item_label.setAlignment(Qt.AlignCenter)
            menu_layout_vbox.addWidget(item_label)

            add_to_cart_button = QPushButton("장바구니 추가")
            add_to_cart_button.setStyleSheet("""
                background-color: #D3D3D3;
                border: 1px solid #A9A9A9;
                border-radius: 5px;
                padding: 5px;
                font-size: 12px;
                min-width: 120px;
            """)
            add_to_cart_button.clicked.connect(lambda checked, o=item, p=price: self.add_to_cart(o, p))
            menu_layout_vbox.addWidget(add_to_cart_button, alignment=Qt.AlignCenter)

            frame.setLayout(menu_layout_vbox)
            self.menu_layout.addWidget(frame, row, col)

            col += 1
            if col >= 3:
                col = 0
                row += 1

    def update_menu(self, category):
        self.current_category = category
        self.display_menu()

    def add_to_cart(self, order, price):
        msg = AddMenu.Request()
        msg.menu = order
        msg.price = price
        future = self.node.order_client.call_async(msg)
        future.add_done_callback(self.add_to_cart_callback)
    
    def add_to_cart_callback(self, future):
        response = future.result()
        order = response.menu
        price = response.price

        #재고 소진 시 주문 불가
        if order == None or price == -1: 
            QMessageBox.warning(self,"재고 부족","재고가 떨어져 주문이 불가능합니다.")
            return

        for idx, (item, _, quantity) in enumerate(self.cart):
            if item == order:
                self.cart[idx] = (item, price, quantity + 1)
                self.update_cart_list()
                return
        self.cart.append((order, price, 1))
        self.update_cart_list()

    def remove_from_cart(self):
        request = CancelMenu.Request()

        selected_items = self.cart_list.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "항목 제거", "제거할 항목을 선택하세요!")
            return
        for item in selected_items:
            order_name = item.text().split(" - ")[0].strip()
            for idx, (cart_item, price, quantity) in enumerate(self.cart):
                request.menu = cart_item
                request.num = 1
                self.node.cancel_client.call_async(request)

                if cart_item == order_name:
                    if quantity > 1:
                        self.cart[idx] = (cart_item, price, quantity - 1)
                    else:
                        del self.cart[idx]
                    break
            break
        self.update_cart_list()

    def update_cart_list(self):
        self.cart_list.clear()
        for item, price, quantity in self.cart:
            self.cart_list.addItem(f"{item} - {quantity}개 - {price * quantity}원")

    def submit_cart(self):
        if not self.cart:
            QMessageBox.warning(self, "장바구니 주문", "장바구니가 비어 있습니다!")
            return

        for order, price, quantity in self.cart:
            msg = String()
            msg.data = f"Table_{self.node.table_id} - {order} ({quantity}개, {price * quantity}원)"
            self.node.publisher_.publish(msg)

            insert_table_info('orders', ['table_id', 'menu', 'price', 'quantity'],[self.node.table_id, order, price, quantity])

        QMessageBox.information(self, "장바구니 주문", "주문이 전송되었습니다!")
        self.cart.clear()
        self.update_cart_list()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("table_id set to table_1 by default")
        table_id = '1'
    else : table_id = sys.argv[1]
    # namespace 생성 (table_1, table_2 등)
    namespace = f"table_{table_id}"

    node = OrderSystemNode(table_id,namespace=namespace)

    app = QApplication(sys.argv)
    gui = OrderSystemGUI(node)
    gui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))
    timer.start(100)

    sys.exit(app.exec_())

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''
def start_ros2_node(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("table_id set to table_1 by default")
        table_id = '1'
    else:
        table_id = sys.argv[1]
    
    # namespace 생성 (table_1, table_2 등)
    namespace = f"table_{table_id}"

    node = OrderSystemNode(table_id, namespace=namespace)

    app = QApplication(sys.argv)
    gui = OrderSystemGUI(node)
    gui.show()

    # MultiThreadedExecutor 설정
    ros2_thread = threading.Thread(target=start_ros2_node, args=(node,))
    ros2_thread.start()
    
    # GUI 실행
    sys.exit(app.exec_())

    node.destroy_node()
    rclpy.shutdown()
    ros2_thread.join()

if __name__ == "__main__":
    main()
'''