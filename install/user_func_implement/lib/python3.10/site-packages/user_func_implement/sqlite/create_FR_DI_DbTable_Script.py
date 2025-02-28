import sqlite3
from sqlite3 import OperationalError


def create_database_create_table():
    conn = sqlite3.connect('fr_data.db')
    cur = conn.cursor()
    try:
        sql = """CREATE TABLE fr_DI_tags (                    
                    name VARCHAR(20) primary key,
                    Tag TEXT not null DEFAULT ''
                    
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