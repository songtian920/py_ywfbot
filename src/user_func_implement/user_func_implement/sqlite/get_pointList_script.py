import sqlite3
from sqlite3 import OperationalError


# sqlite3加载点位数据
def fr_load_point():
    conn = sqlite3.connect('fr_data.db')
    # 创建一个Cursor:
    cursor = conn.cursor()
    try:
        #读取表格信息
        cursor.execute('SELECT * FROM fr_pointData')
        #获取所有值
        values = cursor.fetchall()
        for val in values:
            dic_name = ''
            data_list=[]
            #赋值
            dic_name=val[1]  #数据库中的name
            data_list.append(val[0])  #数据库中的id
            data_list.append(val[2])  #数据库中的comment注释
            data_list.append(val[3])  # 数据库中的joint1
            data_list.append(val[4])  # 数据库中的joint2
            data_list.append(val[5])  # 数据库中的joint3
            data_list.append(val[6])  # 数据库中的joint4
            data_list.append(val[7])  # 数据库中的joint5
            data_list.append(val[8])  # 数据库中的joint6
            data_list.append(val[9])  # 数据库中的cart_x
            data_list.append(val[10])  # 数据库中的cart_y
            data_list.append(val[11])  # 数据库中的cart_z
            data_list.append(val[12])  # 数据库中的cart_xRot
            data_list.append(val[13])  # 数据库中的cart_yRot
            data_list.append(val[14])  # 数据库中的cart_zRot
            data_list.append(val[15])  # 数据库中的tool
            data_list.append(val[16])  # 数据库中的user
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
        cursor.close()
        conn.close()

if __name__ == "__main__":
    fr_load_point()