import QtQuick
import Qt.labs.qmlmodels
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15


Rectangle{
    id: root_userCalib
    width: 800
    height: 500
    color: "#343b48"
    visible: false

    //按钮区域
    Row{
        anchors.top: parent.top
        anchors.topMargin: 20
        anchors.right: parent.right
        anchors.rightMargin: 20
        spacing: 20
        //上一页
        MyButtonText{
            id: last_page_btn
            widthRect: 80
            heightRect: 40
            buttonText: '上一页'
            pointSize: 12
            color:"#3c4454"  //初始设置为主页

            // 鼠标悬浮时的背景色变化
            MouseArea {
                anchors.fill: parent
                hoverEnabled: true // 设置为true以启用悬停事件
                onPressed: {
                    last_page_btn.color = "#dce1ec"
                    //console.log("add_data_btn press")
                }
                onReleased: {
                    last_page_btn.color = "#3c4454"

                }
                onClicked: {
                    if(stackLayout_win.currentIndex >0)
                    {
                        stackLayout_win.currentIndex = stackLayout_win.currentIndex - 1
                    }
                }
            }
        }
        //下一页
        MyButtonText{
            id: next_page_btn
            widthRect: 80
            heightRect: 40
            buttonText: '下一页'
            pointSize: 12
            color:"#3c4454"  //初始设置为主页

            // 鼠标悬浮时的背景色变化
            MouseArea {
                anchors.fill: parent
                hoverEnabled: true // 设置为true以启用悬停事件
                onPressed: {
                    next_page_btn.color = "#dce1ec"
                    //console.log("add_data_btn press")
                }
                onReleased: {
                    next_page_btn.color = "#3c4454"

                }
                onClicked: {
                    if(stackLayout_win.currentIndex <8)
                    {
                        stackLayout_win.currentIndex = stackLayout_win.currentIndex+1
                    }
                }
            }
        }
        //关闭
        MyButtonText{
            id: close_page_btn
            widthRect: 80
            heightRect: 40
            buttonText: '关闭'
            pointSize: 12
            color:"#3c4454"  //初始设置为主页

            // 鼠标悬浮时的背景色变化
            MouseArea {
                anchors.fill: parent
                hoverEnabled: true // 设置为true以启用悬停事件
                onPressed: {
                    close_page_btn.color = "#dce1ec"
                    //console.log("add_data_btn press")
                }
                onReleased: {
                    close_page_btn.color = "#3c4454"

                }
                onClicked: {
                    root_userCalib.visible = false
                }
            }
        }
    }

    //抽屉面板
    StackLayout
    {
        id: stackLayout_win
        anchors.top: parent.top
        anchors.topMargin: 80
        anchors.right: parent.right
        anchors.rightMargin: 10
        anchors.left: parent.left
        anchors.leftMargin: 10
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 10
        visible: true
        currentIndex: 0

        //第一个点位 point0示教
        Rectangle{
            id: point0_rect
            width: parent.width
            height: parent.height
            Layout.alignment: Qt.AlignCenter

            color: "#343b48"
            visible: false

            Column{
                anchors.centerIn: parent
                spacing: 10
                Text {

                    text: name_selected+"：第一个点位示教,保持工具姿势不变，移动机器人末端工具针尖对准用户坐标系o点,\n然后点击示教按钮，获取当前点位坐标，成功后点击 下一页 按钮"
                    font.pixelSize: 15 // 设置字体大小为15像素
                    color: "black"
                    anchors.horizontalCenter: parent.horizontalCenter
                }
                Image {
                    id: point0_Image
                    source: "image/robotUserPoint0.jpg"  // 替换为你的图片路径
                    width: 200
                    height: 180
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                //文本框
                Rectangle {
                    width: 420
                    height: 50
                    color: "lightgray"
                    anchors.horizontalCenter: parent.horizontalCenter

                    Text {
                        id: point0_text
                        text: "请点击示教按钮"
                        font.pixelSize: 15 // 设置字体大小为15像素
                        color: "black"
                        anchors.centerIn: parent // 居中文本
                    }
                }

                //示教按钮
                MyButtonText{
                    id: teach_point0_btn
                    widthRect: 80
                    heightRect: 40
                    buttonText: '示教'
                    pointSize: 12
                    color:"#3c4454"  //初始设置为主页
                    anchors.horizontalCenter: parent.horizontalCenter
                    // 鼠标悬浮时的背景色变化
                    MouseArea {
                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            teach_point0_btn.color = "#dce1ec"
                            //console.log("add_data_btn press")
                        }
                        onReleased: {
                            teach_point0_btn.color = "#3c4454"

                        }
                        onClicked: {
                            signal_teach_user_point(0)
                        }
                    }
                }
            }
        }
        //第二个点位 point1示教
        Rectangle{
            id: point1_rect
            width: parent.width
            height: parent.height
            Layout.alignment: Qt.AlignCenter

            color: "#343b48"
            visible: false

            Column{
                anchors.centerIn: parent
                spacing: 10
                Text {

                    text: name_selected+"：第二个点位示教,保持工具姿势不变，移动机器人末端工具针尖对准用户坐标系x点,\n然后点击示教按钮，获取当前点位坐标，成功后点击 下一页 按钮"
                    font.pixelSize: 15 // 设置字体大小为15像素
                    color: "black"
                    anchors.horizontalCenter: parent.horizontalCenter
                }
                Image {
                    id: point1_Image
                    source: "image/robotUserPoint1.jpg"  // 替换为你的图片路径
                    width: 200
                    height: 180
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                //文本框
                Rectangle {
                    width: 420
                    height: 50
                    color: "lightgray"
                    anchors.horizontalCenter: parent.horizontalCenter

                    Text {
                        id: point1_text
                        text: "请点击示教按钮"
                        font.pixelSize: 15 // 设置字体大小为15像素
                        color: "black"
                        anchors.centerIn: parent // 居中文本
                    }
                }

                //示教按钮
                MyButtonText{
                    id: teach_point1_btn
                    widthRect: 80
                    heightRect: 40
                    buttonText: '示教'
                    pointSize: 12
                    color:"#3c4454"  //初始设置为主页
                    anchors.horizontalCenter: parent.horizontalCenter
                    // 鼠标悬浮时的背景色变化
                    MouseArea {
                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            teach_point1_btn.color = "#dce1ec"

                        }
                        onReleased: {
                            teach_point1_btn.color = "#3c4454"

                        }
                        onClicked: {
                            signal_teach_user_point(1)
                        }
                    }
                }
            }
        }
        //第三个点位 point2示教
        Rectangle{
            id: point2_rect
            width: parent.width
            height: parent.height
            Layout.alignment: Qt.AlignCenter
            color: "#343b48"
            visible: false
            Column{
                anchors.centerIn: parent
                spacing: 10
                Text {
                    text: name_selected+"：第三个点位示教,保持工具姿势不变，移动机器人末端工具针尖对准用户坐标系x点,\n然后点击示教按钮，获取当前点位坐标，成功后点击 下一页 按钮"
                    font.pixelSize: 15 // 设置字体大小为15像素
                    color: "black"
                    anchors.horizontalCenter: parent.horizontalCenter
                }
                Image {
                    id: point2_Image
                    source: "image/robotUserPoint2.jpg"  // 替换为你的图片路径
                    width: 200
                    height: 180
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                //文本框
                Rectangle {
                    width: 420
                    height: 50
                    color: "lightgray"
                    anchors.horizontalCenter: parent.horizontalCenter
                    Text {
                        id: point2_text
                        text: "请点击示教按钮"
                        font.pixelSize: 15 // 设置字体大小为15像素
                        color: "black"
                        anchors.centerIn: parent // 居中文本
                    }
                }
                //示教按钮
                MyButtonText{
                    id: teach_point2_btn
                    widthRect: 80
                    heightRect: 40
                    buttonText: '示教'
                    pointSize: 12
                    color:"#3c4454"  //初始设置为主页
                    anchors.horizontalCenter: parent.horizontalCenter
                    // 鼠标悬浮时的背景色变化
                    MouseArea {
                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            teach_point2_btn.color = "#dce1ec"
                            //console.log("add_data_btn press")
                        }
                        onReleased: {
                            teach_point2_btn.color = "#3c4454"
                        }
                        onClicked: {
                            signal_teach_user_point(2)
                        }
                    }
                }
            }
        }

        //计算出 新工具坐标旋转变换，并更新到界面及数据库
        Rectangle{
            id: userRotate_rect
            width: parent.width
            height: parent.height
            Layout.alignment: Qt.AlignCenter
            color: "#343b48"
            visible: false

            Text {
                anchors.top: parent.top
                anchors.topMargin: 10
                anchors.horizontalCenter: parent.horizontalCenter
                text: name_selected+"：根据前三个（0,x,y）示教的点位计算出新用户坐标系在基坐标系的位姿,\n然后点击更新按钮，将数据更新"
                font.pixelSize: 15 // 设置字体大小为15像素
                color: "black"
            }

            Row{
                anchors.top: parent.top
                anchors.topMargin: 80
                anchors.horizontalCenter: parent.horizontalCenter
                spacing: 10
                Image {
                    id: user_rotate_Image
                    source: "image/calculate_user_rotate.jpg"  // 替换为你的图片路径
                    width: 200
                    height: 180
                    anchors.verticalCenter: parent.verticalCenter
                }
                Column{
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 10
                    //文本框0
                    Rectangle {
                        width: 420
                        height: 35
                        color: "lightgray"
                        anchors.horizontalCenter: parent.horizontalCenter

                        Text {
                            id: pose0_text
                            text: point0_text.text
                            font.pixelSize: 15 // 设置字体大小为15像素
                            color: "black"
                            anchors.centerIn: parent // 居中文本
                        }
                    }

                    //文本框1
                    Rectangle {
                        width: 420
                        height: 35
                        color: "lightgray"
                        anchors.horizontalCenter: parent.horizontalCenter

                        Text {
                            id: pose1_text
                            text: point1_text.text
                            font.pixelSize: 15 // 设置字体大小为15像素
                            color: "black"
                            anchors.centerIn: parent // 居中文本
                        }
                    }
                    //文本框2
                    Rectangle {
                        width: 420
                        height: 35
                        color: "lightgray"
                        anchors.horizontalCenter: parent.horizontalCenter

                        Text {
                            id: pose2_text
                            text: point2_text.text
                            font.pixelSize: 15 // 设置字体大小为15像素
                            color: "black"
                            anchors.centerIn: parent // 居中文本
                        }
                    }
                }
            }

            Row{
                anchors.top: parent.top
                anchors.topMargin: 300
                anchors.horizontalCenter: parent.horizontalCenter
                spacing: 30

                Rectangle {
                    width: 420
                    height: 35
                    color: "lightgray"
                    anchors.verticalCenter: parent.verticalCenter
                    Text {
                        id: calculate_userPose_text
                        text: "计算新工具在默认工具的旋转角度"
                        font.pixelSize: 15 // 设置字体大小为15像素
                        color: "black"
                        anchors.centerIn: parent // 居中文本
                    }
                }

                //计算按钮
                MyButtonText{
                    id: calculate_userPose_btn
                    widthRect: 80
                    heightRect: 40
                    buttonText: '计算'
                    pointSize: 12
                    color:"#3c4454"  //初始设置为主页
                    anchors.verticalCenter: parent.verticalCenter
                    // 鼠标悬浮时的背景色变化
                    MouseArea {
                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            calculate_userPose_btn.color = "#dce1ec"
                            //console.log("add_data_btn press")
                        }
                        onReleased: {
                            calculate_userPose_btn.color = "#3c4454"
                        }
                        onClicked: {
                            //发送计算平移变换信号
                            signal_calculate_user_coord()
                        }
                    }
                }
                //更新到界面及数据库
                MyButtonText{
                    id: update_user_coord_btn
                    widthRect: 80
                    heightRect: 40
                    buttonText: '保存'
                    pointSize: 12
                    color:"#3c4454"
                    anchors.verticalCenter: parent.verticalCenter
                    // 鼠标悬浮时的背景色变化
                    MouseArea {
                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            update_user_coord_btn.color = "#dce1ec"
                        }
                        onReleased: {
                            update_user_coord_btn.color = "#3c4454"
                        }
                        onClicked: {
                            signal_update_user_coord(name_selected)
                        }
                    }
                }
            }
        }
    }

    function teach_point_callBack(list_pose_data,index)
    {
        console.log("teach_point_callBack")
        if(index ==0)
        {
            console.log(String(list_pose_data[0]))
            point0_data = list_pose_data
            point0_text.text = String(list_pose_data[0])+" , "+String(list_pose_data[1])+" , "+String(list_pose_data[2])+
                " , "+String(list_pose_data[3])+" , "+String(list_pose_data[4])+" , "+String(list_pose_data[5])
        }
        else if(index ==1)
        {
            point1_data = list_pose_data
            point1_text.text = String(list_pose_data[0])+" , "+String(list_pose_data[1])+" , "+String(list_pose_data[2])+
                " , "+String(list_pose_data[3])+" , "+String(list_pose_data[4])+" , "+String(list_pose_data[5])
        }
        else if(index ==2)
        {
            point2_data = list_pose_data
            point2_text.text = String(list_pose_data[0])+" , "+String(list_pose_data[1])+" , "+
                String(list_pose_data[2])+" , "+String(list_pose_data[3])+" , "+
                String(list_pose_data[4])+" , "+String(list_pose_data[5])
        }
    }

    //新用户计算结果回传
    function calculate_user_callback(list_data)
    {
         calculate_userPose_text.text = String(list_data[0])+","+String(list_data[1])+","+
            String(list_data[2])+","+String(list_data[3])+","+
            String(list_data[4])+","+String(list_data[5])
    }

    //示教点位
    signal signal_teach_user_point(int point_index)
    //计算新用户位姿变换
    signal signal_calculate_user_coord()
    //更新新用户在基坐标系位姿
    signal signal_update_user_coord(string name_tool)

    property list<double> point0_data: [0.0,0.0,0.0,0.0,0.0,0.0]
    property list<double> point1_data: [0.0,0.0,0.0,0.0,0.0,0.0]
    property list<double> point2_data: [0.0,0.0,0.0,0.0,0.0,0.0]

    property string name_selected: ""
}