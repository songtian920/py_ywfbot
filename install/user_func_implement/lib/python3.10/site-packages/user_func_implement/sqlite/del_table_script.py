import sqlite3
from sqlite3 import OperationalError

def del_table():
    # 连接到数据库（如果不存在，则会创建）
    conn = sqlite3.connect('fr_data.db')
    cursor = conn.cursor()

    # 定义要删除的表名
    table_name = 'fr_DI_tags'

    # 执行删除表的SQL语句
    cursor.execute(f'DROP TABLE IF EXISTS {table_name}')

    # 提交事务
    conn.commit()

    # 关闭连接
    cursor.close()
    conn.close()

if __name__ == "__main__":
        del_table()