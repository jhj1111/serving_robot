import sqlite3
from typing import Any, List, Optional, Union

db_list = ['menu.db', 'ingredient.db', 'menu_ingredient.db', 'orders.db']
table_db = {}

def create_db(conn, table):
    '''table param 정보기반으로 sql table 생성 함수'''
    if isinstance(conn, str) and conn.endswith('.db') : conn = sqlite3.connect(conn)
    cursor = conn.cursor()
    cursor.execute(table)
    conn.commit()
    conn.close()

def drop_table(conn, table):
    '''table param 정보기반으로 sql table 삭제 함수'''
    if isinstance(conn, str) and conn.endswith('.db') : conn = sqlite3.connect(conn)
    cursor = conn.cursor()
    cursor.execute(f'DROP TABLE {table}')
    conn.commit()
    conn.close()

def delete_table(
    table: str,
    where_column: Union[str, List[str]],
    where_value: Union[str, List[str]],
    db: str = None
):
    '''
    table 데이터 삭제
    table : table 이름 ex) 'Menu'
    where_column : 삭제하려는 컬럼명 또는 컬럼명 리스트
    where_value : 삭제 조건에 해당하는 값 또는 값 리스트
    db : 데이터베이스 파일 경로 (default: table.lower() + '.db')
    '''
    # where_column과 where_value를 리스트로 변환
    where_column = [where_column] if isinstance(where_column, str) else where_column
    where_value = [where_value] if isinstance(where_value, str) else where_value

    # 조건문 유효성 검사
    if len(where_column) != len(where_value):
        raise ValueError("where_column과 where_value의 길이가 일치해야 합니다.")

    # 데이터베이스 연결
    try:
        if db:
            conn = sqlite3.connect(db)
        else:
            conn = sqlite3.connect(table.lower() + '.db')

        c = conn.cursor()

        # DELETE 쿼리 생성
        where_clause = ' AND '.join(f"{col} = ?" for col in where_column)
        delete_query = f"DELETE FROM {table} WHERE {where_clause}"

        # 쿼리 실행
        c.execute(delete_query, where_value)
        conn.commit()

        print(f"DELETE 성공: {c.rowcount}개의 행이 삭제되었습니다.")

    except sqlite3.Error as e:
        print(f"DELETE 중 오류 발생: {e}")

    finally:
        # 커서 및 연결 닫기
        if conn:
            c.close()
            conn.close()


def insert_table_info(table_name: str, attributes: Union[str, List[str]], values, allow_insert_if_exists=False):
    '''
    table 정보 추가 또는 갱신
    table_name : table 이름 ex) 'Menu'
    attributes : table 속성, ex) ('menu_name', 'price')
    values : 추가하려는 값, ex) ('americano', 150000)
    allow_insert_if_exists : True인 경우, 기존 데이터를 삭제 후 갱신
    '''
    # 속성 문자열 처리
    if isinstance(attributes, str):
        attributes = [attributes]
    if len(attributes) > 1:
        try:
            atb = '(' + ', '.join(attributes) + ')'
        except TypeError:
            print('테이블 속성 형식 잘못됨')
            return

    # 데이터베이스 연결
    try:
        conn = sqlite3.connect(table_db[table_name])  # table_db가 정의되어 있다고 가정
    except:
        conn = sqlite3.connect(table_name.lower() + '.db')

    c = conn.cursor()

    # 데이터 존재 여부 확인
    placeholders = ' AND '.join(f"{attr} = ?" for attr in attributes)
    check_query = f"SELECT EXISTS(SELECT 1 FROM {table_name} WHERE {placeholders})"
    c.execute(check_query, values)
    value_exists = c.fetchone()[0] == 1

    # 조건에 따른 처리
    # 데이터가 존재하고 allow_insert_if_exists가 False이면 아무 작업도 하지 않음
    if value_exists and not allow_insert_if_exists : pass

    else:
        try:
            if value_exists and allow_insert_if_exists:
                # 기존 데이터 삭제
                delete_query = f"DELETE FROM {table_name} WHERE {placeholders}"
                c.execute(delete_query, values)
                conn.commit()

            # 데이터 삽입
            insert_query = f'INSERT INTO {table_name} {atb} VALUES ({", ".join(["?" for _ in values])})'
            c.execute(insert_query, values)
            conn.commit()
            
        except sqlite3.Error as e:
            print(f"INSERT 중 오류 발생: {e}")
    
    # 커서 및 연결 닫기
    c.close()
    conn.close()

import sqlite3
from typing import Union, List

def update_table_info(
    table_name: str,
    updated_column: Union[str, List[str]],
    values: Union[tuple, List],
    where_column: Union[str, List[str]],
    where_value: Union[tuple, List],
    db=None
):
    '''
    table의 data를 update하는 기능
    table_name : table 이름 ex) 'Menu'
    updated_column : update를 원하는 table column, ex) 'price' 또는 ['price', 'stock_quantity']
    values : 업데이트할 값, ex) 2000 또는 (2000, 50)
    where_column : 조건이 되는 column, ex) 'menu_name' 또는 ['menu_name', 'category']
    where_value : 조건에 해당하는 값, ex) 'americano' 또는 ('americano', 'beverage')
    db : 데이터베이스 파일 경로 (default: table_name.lower() + '.db')
    '''
    # updated_column과 where_column이 문자열이면 리스트로 변환
    updated_column = [updated_column] if isinstance(updated_column, str) else updated_column
    where_column = [where_column] if isinstance(where_column, str) else where_column
    values = [values] if isinstance(values, int) or isinstance(values, str) else values
    where_value = [where_value] if isinstance(where_value, int) or isinstance(where_value, str) else where_value

    # 데이터베이스 연결
    if db:
        conn = sqlite3.connect(db)
    else:
        try:
            conn = sqlite3.connect(table_db[table_name])  # table_db가 정의되어 있다고 가정
        except:
            conn = sqlite3.connect(table_name.lower() + '.db')

    c = conn.cursor()

    # UPDATE 쿼리 생성
    set_clause = ', '.join(f"{col} = ?" for col in updated_column)
    where_clause = ' AND '.join(f"{col} = ?" for col in where_column)
    update_query = f"UPDATE {table_name} SET {set_clause} WHERE {where_clause}"

    # 업데이트 수행
    try:
        c.execute(update_query, tuple(values) + tuple(where_value))
        conn.commit()
        print(f"UPDATE 성공: {c.rowcount}개의 행이 변경되었습니다.")
    except sqlite3.Error as e:
        print(f"UPDATE 중 오류 발생: {e}")
    finally:
        # 커서 및 연결 닫기
        c.close()
        conn.close()

def calculate_profit():
    '''Menu 테이블의 profit 계산 후 업데이트'''
    conn_menu = sqlite3.connect('menu.db')
    conn_ing = sqlite3.connect('ingredient.db')
    conn_menu_ing = sqlite3.connect('menu_ingredient.db')

    c_menu = conn_menu.cursor()
    c_ing = conn_ing.cursor()
    c_menu_ing = conn_menu_ing.cursor()

    # Menu_Ingredient에서 각 메뉴의 재료와 수량 가져오기
    c_menu_ing.execute("SELECT menu_id, ingredient_id, quantity FROM menu_ingredient")
    menu_ingredients = c_menu_ing.fetchall()

    ingredient_costs = {}

    # Ingredient 테이블에서 재료 단가 가져오기
    c_ing.execute("SELECT ingredient_id, cost FROM ingredient")
    for row in c_ing.fetchall():
        ingredient_costs[row[0]] = row[1]

    # Menu 테이블에서 가격 가져오기
    c_menu.execute("SELECT menu_id, price FROM Menu")
    menu_prices = {row[0]: row[1] for row in c_menu.fetchall()}

    # profit 계산
    menu_costs = {}
    for menu_id, ingredient_id, quantity in menu_ingredients:
        menu_costs[menu_id] = menu_costs.get(menu_id, 0) + ingredient_costs[ingredient_id] * quantity

    for menu_id, cost in menu_costs.items():
        profit = menu_prices[menu_id] - cost
        c_menu.execute("UPDATE Menu SET profit = ? WHERE menu_id = ?", (profit, menu_id))

    conn_menu.commit()

    c_menu.close()
    c_ing.close()
    c_menu_ing.close()
    conn_menu.close()
    conn_ing.close()
    conn_menu_ing.close()

def order_db(conn):
    #table_name = 'Orders'
    table_name = 'orders'
    #table = '''
    #    CREATE TABLE IF NOT EXISTS orders (
    #        table_id INTEGER PRIMARY KEY AUTOINCREMENT,
    #        table_number INTEGER NOT NULL,
    #        menu TEXT NOT NULL,
    #        num INTEGER NOT NULL,
    #        time TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
    #    );
    #'''
    table = '''CREATE TABLE IF NOT EXISTS orders (
                        table_id TEXT,
                        menu TEXT,
                        price INTEGER,
                        quantity INTEGER)'''
    table_db[table_name] = 'orders.db'

    create_db(conn, table)

def menu_db(conn):
    #table_name = 'Menu'
    table_name = 'menu'
    table = '''
        CREATE TABLE IF NOT EXISTS menu (
            menu_id INTEGER PRIMARY KEY AUTOINCREMENT,
            name VARCHAR(100) NOT NULL,
            price DECIMAL(10, 2) NOT NULL,
            category VARCHAR(100) NOT NULL,
            profit DECIMAL(10, 2) DEFAULT NULL
        );
    '''
    table_db[table_name] = 'menu.db'

    create_db(conn, table)

    attributes = ['menu_id', 'name', 'price','category', 'profit']
    menu_name = [
        '아메리카노', '카페모카', '바닐라 라떼', 
        '초콜릿 브라우니', '바닐라 크로와상', '레몬 파운드케이크', 
        '초콜릿 쉐이크', '바닐라 쉐이크', '민트 초코 쉐이크'
    ]
    menu_id = list(range(1, len(menu_name)+1))
    price = [
        3000, 4500, 4500,
        7000, 6000, 5500,
        5500, 6000, 5000
    ]
    category = 3*['음료'] + 3*['디저트'] + 3*['쉐이크']

    for id, name, p, c in zip(menu_id, menu_name, price,category):
        insert_table_info(table_name, attributes[:-1], (id, name, p, c))

def ingredient_db(conn):
    #table_name = 'Ingredient'
    table_name = 'ingredient'
    table = '''
        CREATE TABLE IF NOT EXISTS ingredient (
            ingredient_id INTEGER PRIMARY KEY AUTOINCREMENT,
            ingredient_name VARCHAR(100) NOT NULL,
            cost DECIMAL(10, 2) NOT NULL,
            stock_quantity INT NOT NULL DEFAULT 0
        );
    '''
    table_db[table_name] = 'ingredient.db'

    create_db(conn, table)

    attributes = ['ingredient_id', 'ingredient_name', 'cost', 'stock_quantity']
    ingredient_name = ['에스프레소', '우유', '초콜릿', '바닐라', '설탕', '버터', '밀가루', '아이스크림']
    ingredient_id = list(range(1, len(ingredient_name)+1))
    cost = [200, 150, 300, 250, 50, 400, 100, 500]
    stock_quantity = [50] * len(ingredient_name)

    for id, name, c, qty in zip(ingredient_id, ingredient_name, cost, stock_quantity):
        insert_table_info(table_name, attributes, (id, name, c, qty))

def menu_ingredient_db(conn):
    #table_name = 'Menu_Ingredient'
    table_name = 'menu_ingredient'
    table = '''
        CREATE TABLE IF NOT EXISTS menu_ingredient (
            menu_id INT NOT NULL,
            ingredient_id INT NOT NULL,
            quantity INT NOT NULL,
            PRIMARY KEY (menu_id, ingredient_id),
            FOREIGN KEY (menu_id) REFERENCES Menu(menu_id) ON DELETE CASCADE,
            FOREIGN KEY (ingredient_id) REFERENCES Ingredient(ingredient_id) ON DELETE CASCADE
        );
    '''
    table_db[table_name] = 'menu_ingredient.db'

    create_db(conn, table)

    menu_ingredient_data = [
        (1, 1, 8), # 아메리카노
        (2, 1, 3), (2, 2, 4), (2, 3, 5), # 카페모카
        (3, 1, 3), (3, 2, 5), (3, 4, 3), # 바닐라 라떼
        (4, 3, 8), (4, 5, 8), (4, 6, 3), (4, 7, 5), # 초콜릿 브라우니
        (5, 4, 2), (5, 5, 5), (5, 6, 5), (5, 7, 3), # 바닐라 크로와상
        (6, 5, 3), (6, 6, 5), (6, 7, 8), # 레몬 파운드케이크
        (7, 3, 6), (7, 2, 8), (7, 8, 4), # 초콜릿 쉐이크
        (8, 4, 8), (8, 2, 7), (8, 8, 5), # 바닐라 쉐이크
        (9, 4, 5), (9, 3, 3), (9, 8, 3)  # 민트 초코 쉐이크
    ]

    attributes = ['menu_id', 'ingredient_id', 'quantity']
    for data in menu_ingredient_data:
        insert_table_info(table_name, attributes, data)

def load_db(dbs: List[str] =None):
    '''
    db 생성
    db list : menu_db, ingredient_db, menu_ingredient_db, order_db
    '''
    if dbs is None:
        dbs = db_list

    init_db_funcs = [menu_db, ingredient_db, menu_ingredient_db, order_db]
    for db in dbs:
        table = db.split('.')[0]
        
        conn = sqlite3.connect(db)
        try : drop_table(conn, table)
        except sqlite3.OperationalError : conn.close()
        conn = sqlite3.connect(db)
        init_db_funcs[db_list.index(db)](conn)

def get_table_info(table: str, attributes: str|Optional[List[str]]=None, db: str=None, args: str=None)->List[Any]:
    '''
    설정한 db table의 정보를 가져온다.
    table : table name
    attributes : 정보를 가져오고자 하는 table의 속성
    db : datbase name
    args : 추가 조건 ex) WHERE name == 'aaa'
    '''
    #속성값 입력 안할 시 모든 데이터 가져옴
    attributes = '*' if attributes == None else attributes
    #str 혹은 iter
    attributes = ', '.join(attributes) if not isinstance(attributes, str) else attributes
    db = table.lower() + '.db' if db == None else db
    
    conn = sqlite3.connect(db)
    c = conn.cursor()
    # 쿼리 실행
    query = f"SELECT {attributes} FROM {table}"
    if args is not None:
        query += f" {args}"
    c.execute(query)
    # 데이터 가져오기
    data = c.fetchall()
    
    c.close()
    conn.close()
    return data

def custom_command(db, args:str, isreturn:bool=False):
    '''
    명령어 직접입력
    isreturn : return값 존재여부
    '''
    # 데이터베이스 연결
    try:
        conn = sqlite3.connect(db)
    except:
        conn = sqlite3.connect(db.lower() + '.db')

    c = conn.cursor()

    c.execute(args)
    
    try : return c.fetchall()
    except : pass
    finally :
        c.close()
        conn.close()

def save_recommended_menu():
    # max profit 메뉴 추출
    max_profit_menu = get_table_info(
        table='menu',
        attributes=['name', 'price'],
        db='menu.db',
        args="WHERE profit = (SELECT MAX(profit) FROM menu)"
    )

    if max_profit_menu:
        # max profit 메뉴 정보
        menu_name, menu_price = max_profit_menu[0]

        # orders 테이블에서 '추천' 항목 조회
        existing_recommendation = get_table_info(
            table='orders',
            attributes=['table_id'],
            db='orders.db',
            args="WHERE table_id = '추천'"
        )

        if existing_recommendation:
            # '추천' 항목이 있다면 갱신
            update_table_info(
                table_name='orders',
                updated_column=['menu', 'price', 'quantity'],
                values=(menu_name, menu_price, 0),
                where_column='table_id',
                where_value='추천',
                db='orders.db'
            )
        else:
            # '추천' 항목이 없다면 추가
            insert_table_info(
                table_name='orders',
                attributes=['table_id', 'menu', 'price', 'quantity'],
                values=('추천', menu_name, menu_price, 0),
                allow_insert_if_exists=False
            )

        print(f"추천 메뉴: {menu_name}, 가격: {menu_price}, 추가 또는 갱신 완료")


# DB 생성 및 profit 계산 실행
#load_db(['menu.db', 'menu_ingredient.db', 'ingredient.db'])
#calculate_profit()
#name, price, menu_id = zip(*get_table_info('menu', ['name', 'price', 'menu_id']))
#print(menu_id)
#print(name)
#print(price)
#print(*get_table_info('menu', ['name', 'price']))
#profit = get_table_info('menu', 'profit')
#profit = [i[0] for i in profit]
#print(profit)