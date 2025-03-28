import rclpy
from rclpy.node import Node
from delivery_interfaces.msg import MenuRecommend
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from typing import Any, List, Optional, Union, Dict
from kiosk.dbfuncs import *
import random

class Recommended(Node):
    def __init__(self):
        super().__init__('recommended')
        self.declare_parameter('table_num', 9)
        self.table_num = self.get_parameter('table_num').value
        self.QOS_RKL10T = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


        #재료 기본값
        self.ingredient_default_cost = get_table_info('ingredient', 'cost')
        self.ingredient_default_cost = [i[0] for i in self.ingredient_default_cost]

        # 재료&메뉴 개수
        self.num_menu = len(get_table_info('menu'))
        self.num_ingredient = len(get_table_info('ingredient'))
        
        self.recommend_pub_list = []
        for i in range(1, self.table_num+1):
            self.recommend_pub_list.append(self.create_publisher(MenuRecommend, f'/table_{i}/menu_recommend', self.QOS_RKL10T))

        self.get_logger().info('recommended_Node is ready.')

        # 10초 간격으로 random_ingredient_cost 실행
        self.timer = self.create_timer(10.0, self.random_ingredient_cost)

    def random_ingredient_cost(self):
        #재료 가격 범위
        cost = self.ingredient_default_cost
        cost_min = [i-100 for i in cost]
        cost_max = [i+100 for i in cost]
        
        #가격 갱신
        new_cost = []
        for i in range(self.num_ingredient):
            c = random.randint(cost_min[i], cost_max[i])
            #최소 30원
            c = 30 if c < 30 else c
            new_cost.append(c)

        for i, cost in enumerate(new_cost, start=1):
            update_table_info('ingredient', 'cost', cost, 'ingredient_id', i)

        #메뉴 순이익 갱신
        calculate_profit()
        self.call_recommended_topic()
        self.get_logger().info("Ingredient costs and menu profits updated.")

    def call_recommended_topic(self):
        """
        recommended_dish 토픽 호출
        """
        msg = MenuRecommend()
        menu_and_profits = get_table_info('menu', ['name', 'profit'])

        max_profit = float('-inf')
        #-1 : -순이익 메뉴, 1 : 최고 순이익 메뉴
        recommend = []
        r = ''
        #최고,마이너스 순이익 메뉴 확인
        for m, p in menu_and_profits:
            if p < 0 :
                recommend.append((m, -1))
            else :
                if p > max_profit : 
                    max_profit = p
                    r = m
        
        recommend.append((r, 1))

        # 요청 데이터 준비
        if recommend:
            msg.menu, msg.isrecommend = zip(*recommend)
        else:
            msg.menu, msg.isrecommend = [], []

        for i in range(self.table_num):
            self.recommend_pub_list[i].publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Recommended()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
