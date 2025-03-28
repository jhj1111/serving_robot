import rclpy
from rclpy.node import Node
from delivery_interfaces.srv import CancelMenu, AddMenu
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from kiosk.dbfuncs import *
from datetime import datetime
import sys

class StockChecker(Node):
    def __init__(self, table_id, namespace='table_1'):
        super().__init__('stock_checker', namespace=namespace)

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        #load_db()   #db 세팅
        
        # 서비스 서버 생성
        self.order_server = self.create_service(AddMenu, 'check_stock', self.check_stock, qos_profile=QOS_RKL10V)
        self.cancle_server = self.create_service(CancelMenu, 'menu_cancel',self.menu_cancel_callback, qos_profile=QOS_RKL10V)

    def check_stock(self, request, response):
        """
        check_stock 서비스를 호출하여 재료 재고를 확인하고 응답을 반환.
        #Requset
        string menu
        int32 price
        ---
        #Response
        string menu
        int32 price
        """
        # 요청받은 메뉴 이름
        menu = request.menu
        response.menu = menu
        price = request.price
        response.price = price

        try:
            # menu 테이블에서 해당 메뉴의 menu_id 가져오기
            menu_id = get_table_info('menu', 'menu_id', args=f'WHERE name = "{menu}"')[0][0]

            # menu_ingredient 테이블에서 해당 메뉴에 필요한 재료와 수량 가져오기
            recipe = get_table_info('menu_ingredient', ['ingredient_id', 'quantity'], args=f'WHERE menu_id = {menu_id}')

            # 재료 부족 여부 체크
            for ingredient_id, required_quantity in recipe:
                # ingredient 테이블에서 재고(stock_quantity) 가져오기
                stock_quantity = get_table_info('ingredient', 'stock_quantity', args=f'WHERE ingredient_id = {ingredient_id}')[0][0]
                
                # 재고 부족 여부 확인
                if stock_quantity < required_quantity:
                    response.price = -1
                    break
            else :
                # 재료가 충분한 경우 재료 사용량을 차감하여 재고 갱신
                for ingredient_id, required_quantity in recipe:
                    stock_quantity = get_table_info('ingredient', 'stock_quantity', args=f'WHERE ingredient_id = {ingredient_id}')[0][0]
                    new_stock = stock_quantity - required_quantity
                    update_table_info('ingredient', 'stock_quantity', new_stock, 'ingredient_id', ingredient_id)

        except IndexError:
            # 메뉴가 없는 경우 처리
            response.menu = None
        except Exception as e:
            # 기타 오류 처리
            response.menu = None
            self.get_logger().info(f"Error while checking stock: {str(e)}")

        return response

    def menu_cancel_callback(self, request, response):
        """
        메뉴 캔슬 서비스 서버
        """
        menu = request.menu
        num_calcled = request.num

        try:
            # menu 테이블에서 해당 메뉴의 menu_id 가져오기
            menu_id = get_table_info('menu', 'menu_id', args=f'WHERE name = "{menu}"')[0][0]

            # menu_ingredient 테이블에서 해당 메뉴에 필요한 재료와 수량 가져오기
            recipe = get_table_info('menu_ingredient', ['ingredient_id', 'quantity'], args=f'WHERE menu_id = {menu_id}')

             # 재료 재고량 복구
            for ingredient_id, required_quantity in recipe:
                stock_quantity = get_table_info('ingredient', 'stock_quantity', args=f'WHERE ingredient_id = {ingredient_id}')[0][0]
                new_stock = stock_quantity + num_calcled * required_quantity
                update_table_info('ingredient', 'stock_quantity', new_stock, 'ingredient_id', ingredient_id)
                #insert_table_info('ingredient', ['ingredient_id', 'stock_quantity'], (ingredient_id, new_stock), allow_insert_if_exists=True)

        except IndexError:
            # 메뉴가 없는 경우 처리
            self.get_logger().info("no menu")
        except Exception as e:
            # 기타 오류 처리
            self.get_logger().info(f"Error while checking stock: {str(e)}")

        return response

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("table_id set to table_1 by default")
        table_id = '1'
    else : table_id = sys.argv[1]
    # namespace 생성 (table_1, table_2 등)
    namespace = f"table_{table_id}"
    node = StockChecker(table_id,namespace=namespace)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
