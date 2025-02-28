import sqlite3
from sqlite3 import OperationalError


def insert_delete_update_select():
    conn = sqlite3.connect('fr_data.db')
    cur = conn.cursor()
    try:
        # 插入单条语句
        name_list = ["0","1","2","3","4","5","6","7","8","9","10","11","12","13","14","15"]
        for name_ in name_list:
            cur.execute('INSERT INTO fr_DO_tags (name) VALUES (?)', (name_, ))

        conn.commit()
    except Exception as e:
        print(str(e))
    finally:
        cur.close()
        conn.close()


if __name__ == "__main__":
    insert_delete_update_select()