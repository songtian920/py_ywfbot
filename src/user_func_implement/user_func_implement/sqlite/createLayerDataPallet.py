import sqlite3
from sqlite3 import OperationalError


def create_database_create_table():
    conn = sqlite3.connect('palletizingProcess.db')
    cur = conn.cursor()
    try:
        sql = """CREATE TABLE layerData (                    
                    layer INT primary key,
                    layerMode TEXT not null DEFAULT '选择模型',
                    layerHeight FLOAT not null DEFAULT 0,
                    x_offset FLOAT not null DEFAULT 0,
                    y_offset FLOAT not null DEFAULT 0
                );"""
        cur.execute(sql)
        print("create table success")
        return True
    except OperationalError as o:
        print(str(o))
        pass
        if str(o) == "table gas_price already exists":
            return True
        return False
    except Exception as e:
        print(e)
        return False
    finally:
        cur.close()
        conn.close()


if __name__ == "__main__":
    create_database_create_table()