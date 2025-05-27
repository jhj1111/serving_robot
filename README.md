# Kiosk & Delivery Robot Demo (ROS 2 Humble)

## 개요
소규모 카페·레스토랑 환경을 가정해  
- **테이블 주문용 키오스크**  
- **주방 주문 현황판**  
- **메뉴 추천 시스템**  
- **자율주행 서빙 로봇**  

을 모두 ROS 2 노드로 구성한 데모 패키지입니다.

| 패키지 | 언어 | 역할 |
|--------|------|------|
| **kiosk** | Python / PyQt5 | GUI - 주문, 재고 확인, 주방 디스플레이, 추천 |
| **delivery_interfaces** | IDL | 주문 추가·취소 서비스, 추천 메뉴 메시지 |
| **robot_move** | Python | Gazebo, Rviz2 로봇 제어 |

---

## 디렉토리 구조

src/   
├── kiosk/ # PyQt5 기반 GUI 노드   
│ ├── launch/ # order.launch.py, kitchen.launch.py   
│ ├── images/ # 메뉴 이미지 (*.jpeg)   
│ └── kiosk/   
│ │      ├── kitchen/ # 주문·재고 노드   
│ │      └── order/  # 주방 디스플레이·추천 노드    
│ ├── dbfuncs.py # SQLite3 유틸   
│ └── robot_move/ # Gazebo, Rviz2 로봇 제어 노드   
├── delivery_interfaces/ # msg / srv 정의   
└── delivery/ 

---

## 의존성

| 종류 | 패키지 |
|------|--------|
| ROS 2 | Humble Hawksbill |
| Python | rclpy, PyQt5 |
| 기타 | SQLite3(표준), ament_cmake / ament_python |


## 실행 방법
1) 테이블 주문 + 재고 확인
```bash
ros2 launch kiosk order.launch.py table_num:=4   # 1~4번 테이블 GUI
```
> 노드	기능	주요 통신
- order_system_gui	테이블별 주문·결제 GUI	add_menu srv 클라이언트
- order_checker	재고 부족 알림	menu.db, ingredient.db 직접 접근

2) 주방 디스플레이 + 추천 퍼블리셔
```bash
ros2 launch kiosk kitchen.launch.py log_level:=info
```

> 노드	기능	주요 통신
- kitchen_display_gui	완료/대기 주문 보드	cancel_menu srv 클라이언트
- recommended	무작위 추천 메뉴 broadcast	/menu_recommend topic

3) Gazebo, Rviz 및 로봇 제어
```bash
ros2 launch robot_move robot_move.launch.py
```

## 토픽·서비스·파라미터
> 타입	이름	메시지/서비스	발신/수신
- Topic	/menu_recommend	delivery_interfaces/msg/MenuRecommend	recommended → 모든 GUI
- Service	add_menu	delivery_interfaces/srv/AddMenu	주문 GUI → 주방
- Service	cancel_menu	delivery_interfaces/srv/CancelMenu	주방 GUI → 주문 GUI

## 노드	파라미터	기본값	설명
- order_system_gui	table_id	1	테이블 번호   
- order_checker	table_id	1	↑ 동일   
- recommended	table_num	9	추천 대상 최대 테이블 수

## 데이터베이스 스키마
- menu.db : 메뉴(name, price, category, stock)

- ingredient.db : 재료(name, amount, cost)

- menu_ingredient.db : 메뉴-재료 매핑

- orders.db : 주문 기록(table, menu, qty, status)

- kiosk/dbfuncs.py 의 헬퍼 함수 호출 시 자동 생성됩니다.
