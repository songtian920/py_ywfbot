import sqlite3
from sqlite3 import OperationalError


def insert_delete_update_select():
    conn = sqlite3.connect('fr_data.db')
    cur = conn.cursor()
    try:
        # 插入单条语句

        cur.execute('INSERT INTO fr_pointData (name) VALUES (?)', ('p3', ))

        conn.commit()
    except Exception as e:
        print(str(e))
    finally:
        cur.close()
        conn.close()


if __name__ == "__main__":
    insert_delete_update_select()