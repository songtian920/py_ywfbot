import sqlite3
from sqlite3 import OperationalError


def create_database_create_table():
    conn = sqlite3.connect('palletizingProcess.db')
    cur = conn.cursor()
    try:
        sql = """CREATE TABLE param_palletProcess (                    
                    name VARCHAR(20) primary key,
                    comment TEXT not null DEFAULT '注释',
                    tool TEXT not null DEFAULT "tool0",
                    user TEXT not null DEFAULT "fr_baseLink",
                    tool_x_offset FLOAT not null DEFAULT 0,
                    tool_y_offset FLOAT not null DEFAULT 0,
                    tool_z_offset FLOAT not null DEFAULT 0,
                    tool_xRot_offset FLOAT not null DEFAULT 0,
                    tool_yRot_offset FLOAT not null DEFAULT 0,
                    tool_zRot_offset FLOAT not null DEFAULT 0,
                    user_x_offset FLOAT not null DEFAULT 0,
                    user_y_offset FLOAT not null DEFAULT 0,
                    user_z_offset FLOAT not null DEFAULT 0,
                    user_xRot_offset FLOAT not null DEFAULT 0,
                    user_yRot_offset FLOAT not null DEFAULT 0,
                    user_zRot_offset FLOAT not null DEFAULT 0,
                    passPointName  TEXT not null DEFAULT "",
                    palletLength FLOAT not null DEFAULT 0,
                    palletWidth FLOAT not null DEFAULT 0,
                    pixelScale FLOAT not null DEFAULT 0
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