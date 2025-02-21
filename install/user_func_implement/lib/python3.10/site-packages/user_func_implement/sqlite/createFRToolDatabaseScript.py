import sqlite3
from sqlite3 import OperationalError


def create_database_create_table():
    conn = sqlite3.connect('fr_data.db')
    cur = conn.cursor()
    try:
        sql = """CREATE TABLE fr_ToolData (                    
                    name VARCHAR(20) primary key,
                    comment TEXT not null DEFAULT '注释',
                    cart_x FLOAT not null DEFAULT 0,
                    cart_y FLOAT not null DEFAULT 0,
                    cart_z FLOAT not null DEFAULT 0,
                    cart_xRot FLOAT not null DEFAULT 0,
                    cart_yRot FLOAT not null DEFAULT 0,
                    cart_zRot FLOAT not null DEFAULT 0   
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