import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

//机器人手动按钮区
ColumnLayout{
    id: robot_manual_layout
    spacing:30

    //坐标系单选框
    Row {
        Layout.alignment: Qt.AlignLeft
        spacing: 25
        RadioButton {
            id: jointSpace_RBtn
            text: qsTr("轴角度")
            checked: true // 默认选中第一个选项
            onCheckedChanged: {
                if (checked)
                {
                    workSpace_select=0
                    signal_WorkSpaceChange(workSpace_select)
                }
            }
        }
        RadioButton {
            id: baseSpace_RBtn
            text: qsTr("基坐标系")
            onCheckedChanged: {
                if (checked)
                {   workSpace_select=1
                    signal_WorkSpaceChange(workSpace_select)
                }
            }
        }
        RadioButton {
            id: toolSpace_RBtn
            text: qsTr("工具坐标系")
            onCheckedChanged: {
                if (checked)
                {
                    workSpace_select=2
                    signal_WorkSpaceChange(workSpace_select)
                }
            }
        }
        RadioButton {
            id: wObjSpace_RBtn
            text: qsTr("用户坐标系")
            onCheckedChanged: {
                if (checked)
                {
                    workSpace_select=3
                    signal_WorkSpaceChange(workSpace_select)
                }
            }
        }
    }

    //工具坐标系 和用户坐标系
    Row{
        Layout.alignment: Qt.AlignLeft
        spacing: 30

        //机器人上电按钮
        MyButtonText{
            id:serverOn_btn
            widthRect: 100
            heightRect: 40
            //Layout.alignment: Qt.AlignVCenter
            //anchors.verticalCenter: parent.verticalCenter
            buttonText:'机器人使能'
            pointSize:12
            color:"green"  //初始设置为主页
            MouseArea {
                anchors.fill: parent
                hoverEnabled: false // 设置为true以启用悬停事件
                onPressed: {

                }
                onReleased: {

                }
                onClicked: {
                    if(serverOnOff_status)
                    {
                        signal_robotEnable(0)  //机器人使能
                        serverOn_btn.color = "gray"
                        serverOnOff_status = false
                    }
                    else
                    {
                        signal_robotEnable(1)  //机器人使能
                        serverOn_btn.color = "green"
                        serverOnOff_status = true
                    }
                }
            }
        }
        //工具坐标系
        Row{
            Layout.alignment: Qt.AlignVCenter
            spacing: 5
            Text {
                text: '工具坐标系' // 绑定到外部传入的属性
                font.pointSize: 12
                color: "white" // 字体颜色为白色
                Layout.alignment: Qt.AlignVCenter
            }
            //工具坐标系选择
            ComboBox {
                id: tool_comboBox
                width: 150
                height:30
                Layout.alignment: Qt.AlignVCenter
                model: toolListMolde // 下拉框的选项

                onCurrentIndexChanged: {
                    console.log(tool_comboBox.currentText)
                    console.log(tool_comboBox.currentIndex)
                    if (currentIndex !== -1)
                    {
                        //获取选框选择项的名称
                        var tool_name_current = tool_comboBox.model.get(currentIndex).name
                        signal_toolChange(tool_name_current)
                    }
                }
            }

            ListModel {
                id: toolListMolde
                //ListElement {name: "aaaa"; type: "AAAA"}
                //ListElement {name: "bbbb"; type: "BBBB"}
            }
        }
        //用户坐标系
        Row{
            spacing: 5
            Layout.alignment: Qt.AlignVCenter
            Text {
                text: '用户坐标系' // 绑定到外部传入的属性
                font.pointSize: 12
                color: "white" // 字体颜色为白色
                Layout.alignment: Qt.AlignVCenter
            }

            ComboBox {
                id: user_comboBox
                width: 150
                height:30
                Layout.alignment: Qt.AlignVCenter
                model: userListMolde // 下拉框的选项

                onCurrentIndexChanged: {
                    // 当下拉框的选项变化时，这里可以添加处理代码
                    console.log(user_comboBox.currentText)
                    console.log(user_comboBox.currentIndex)
                    if (currentIndex !== -1)
                    {
                        //获取选框选择项的名称
                        var user_name_current = user_comboBox.model.get(currentIndex).name
                        signal_userChange(user_name_current)
                    }

                }
            }
            ListModel {
                id: userListMolde
                //ListElement {name: "aaaa"; type: "AAAA"}
                //ListElement {name: "bbbb"; type: "BBBB"}
            }
        }
    }

    //点位示教及手动点动
    Row{
        spacing: 50
        Layout.alignment: Qt.AlignLeft
        //点位示教 显示块
        Column{
            spacing: 60
            Layout.alignment: Qt.AlignVCenter

            //机器人位置显示
            Grid {
                id: grid
                //width: 200
                //height: 200
                rows: 2
                columns: 3
                spacing: 20
                Layout.alignment: Qt.AlignHCenter
                //1轴
                Column{
                    spacing:10

                    Text {
                            Layout.alignment: Qt.AlignHCenter
                            text: _robot_j1_xPos_text // 绑定到外部传入的属性
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                    }

                    Rectangle{
                        width: 80
                        height: 30
                        color: "#21252d" // 默认背景
                        radius: 10
                        border.color: "transparent"
                        Layout.alignment: Qt.AlignHCenter

                        Text {
                            text: _robot_j1_xPos_value.toFixed(3)
                            font.pointSize:10
                            color: "white" // 字体颜色为白色
                            anchors.centerIn: parent
                        }
                    }
                }
                //2轴
                Column{
                    spacing:10

                    Text {
                            Layout.alignment: Qt.AlignHCenter
                            text: _robot_j2_yPos_text // 绑定到外部传入的属性
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                    }

                    Rectangle{
                        width: 80
                        height: 30
                        color: "#21252d" // 默认背景
                        radius: 10
                        border.color: "transparent"
                        Layout.alignment: Qt.AlignHCenter

                        Text {
                            text: _robot_j2_yPos_value.toFixed(3)
                            font.pointSize:10
                            color: "white" // 字体颜色为白色
                            anchors.centerIn: parent
                        }
                    }
                }
                //3轴
                Column{
                    spacing:10

                    Text {
                            Layout.alignment: Qt.AlignHCenter
                            text: _robot_j3_zPos_text // 绑定到外部传入的属性
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                    }

                    Rectangle{
                        width: 80
                        height: 30
                        color: "#21252d" // 默认背景
                        radius: 10
                        border.color: "transparent"
                        Layout.alignment: Qt.AlignHCenter

                        Text {
                            text: _robot_j3_zPos_value.toFixed(3)
                            font.pointSize:10
                            color: "white" // 字体颜色为白色
                            anchors.centerIn: parent
                        }
                    }
                }
                //4轴
                Column{
                    spacing:10

                    Text {
                            Layout.alignment: Qt.AlignHCenter
                            text: _robot_j4_xRot_text // 绑定到外部传入的属性
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                    }

                    Rectangle{
                        width: 80
                        height: 30
                        color: "#21252d" // 默认背景
                        radius: 10
                        border.color: "transparent"
                        Layout.alignment: Qt.AlignHCenter

                        Text {
                            text: _robot_j4_xRot_value.toFixed(3)
                            font.pointSize:10
                            color: "white" // 字体颜色为白色
                            anchors.centerIn: parent
                        }
                    }
                }
                //5轴
                Column{
                    spacing:10

                    Text {
                            Layout.alignment: Qt.AlignHCenter
                            text: _robot_j5_yRot_text // 绑定到外部传入的属性
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                    }

                    Rectangle{
                        width: 80
                        height: 30
                        color: "#21252d" // 默认背景
                        radius: 10
                        border.color: "transparent"
                        Layout.alignment: Qt.AlignHCenter

                        Text {
                            text: _robot_j5_yRot_value.toFixed(3)
                            font.pointSize:10
                            color: "white" // 字体颜色为白色
                            anchors.centerIn: parent
                        }
                    }
                }
                //6轴
                Column{
                    spacing:10

                    Text {
                            Layout.alignment: Qt.AlignHCenter
                            text: _robot_j6_zRot_text // 绑定到外部传入的属性
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                    }

                    Rectangle{
                        width: 80
                        height: 30
                        color: "#21252d" // 默认背景
                        radius: 10
                        border.color: "transparent"
                        Layout.alignment: Qt.AlignHCenter

                        Text {
                            text: _robot_j6_zRot_value.toFixed(3)
                            font.pointSize:10
                            color: "white" // 字体颜色为白色
                            anchors.centerIn: parent
                        }
                    }
                }
            }

            //点位执行区块
            Column{
                spacing:15
                Layout.alignment: Qt.AlignHCenter
                Row{
                    spacing: 50
                    Layout.alignment: Qt.AlignHCenter
                    //点位选择框
                    ComboBox {
                        id: point_comboBox
                        width: 160
                        height: 40
                        Layout.alignment: Qt.AlignHCenter
                        model: robot_pointMolde
                        onCurrentIndexChanged: {
                            console.log(point_comboBox.currentText)
                            if(point_comboBox.currentIndex != -1)
                            {
                                point_comment_current = point_comment_Molde.get(point_comboBox.currentIndex).name
                            }
                        }
                    }
                    ListModel {
                        id: robot_pointMolde
                        //ListElement {name: "aaaa"; type: "AAAA"}
                        //ListElement {name: "bbbb"; type: "BBBB"}
                    }
                    Text
                     {
                        id: point_comment_text
                        text: point_comment_current  //
                        font.pointSize: 12
                        color: "green" // 字体颜色为白色
                    }
                    ListModel {
                        id: point_comment_Molde
                        //ListElement {name: "aaaa"; type: "AAAA"}
                        //ListElement {name: "bbbb"; type: "BBBB"}
                    }
                }

                Row{
                    spacing: 50
                    Layout.alignment: Qt.AlignHCenter
                    ComboBox {
                        id: moveType_comboBox
                        width: 160
                        height:40
                        Layout.alignment: Qt.AlignVCenter
                        model: ["MoveJoint","MovePose","MoveL_Joint","MoveL_Pose"]
                        currentIndex: 0
                        onCurrentIndexChanged: {
                            console.log(moveType_comboBox.currentText)
                        }
                    }

                    //执行点位按钮
                    MyButtonText{
                        id:pointMove_btn
                        widthRect: 80
                        heightRect: 40
                        buttonText:'执行点位'
                        pointSize:12
                        color:"#3c4454"  //初始设置为主页
                        Layout.alignment: Qt.AlignVCenter
                        MouseArea {
                            anchors.fill: parent
                            hoverEnabled: false // 设置为true以启用悬停事件
                            onPressed: {
                                pointMove_btn.color = "#dce1ec"

                                //获得选取点位名称
                                var point_name = point_comboBox.currentText
                                var moveType = moveType_comboBox.currentText
                                //发送点位运动信号
                                signal_pointMove(point_name,moveType,1)
                            }
                            onReleased: {
                                pointMove_btn.color = "#3c4454"
                                //发送点位运动信号
                                signal_pointMove("","",0)
                            }
                            onClicked: {
                                //console.log("clicked")
                            }
                        }
                    }
                }
            }
        }

        //手动按钮区
        Column{
            spacing:18
            //J1轴，x轴
            RowLayout{
                id: j1_or_xAxis  //JOG J1轴或x轴
                Layout.alignment: Qt.AlignVCenter
                spacing:25
                //减小按钮
                MyButtonText{
                    id: j1_x_RVS_btn
                    widthRect: 50
                    heightRect: 50
                    buttonText:'-'
                    pointSize:25
                    color:"#3c4454"  //初始设置为主页

                    // 鼠标悬浮时的背景色变化
                    MouseArea {
                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            j1_x_RVS_btn.color = "#dce1ec"
                            //信号测试
                            //robot_manual_layout.signal_test()
                            //发送点动信号1轴，反向
                            signal_StartJOG(1,-1)
                            console.log("signal_StartJOG")
                        }
                        onReleased: {
                            j1_x_RVS_btn.color = "#3c4454"
                            //停止点动
                            signal_StopJOG()
                            console.log("signal_StopJOG")
                        }
                        onClicked: {
                            console.log("clicked")
                        }
                    }
                }

                Text
                 {
                    text: "J1/XPos"  //
                    font.pointSize: 15
                    color: "#c3ccdf" // 字体颜色为白色
                }

                //增大按钮
                MyButtonText{
                    id: j1_x_FWD_btn
                    widthRect: 50
                    heightRect: 50
                    buttonText:'+'
                    pointSize:25
                    color:"#3c4454"  //初始设置为主页

                    // 鼠标悬浮时的背景色变化
                    MouseArea {
                        id: mouseArea_j1_x_FWD_btn
                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onEntered: {
                            // 鼠标进入区域时，按钮颜色变化
                            color = "#2c313c"
                        }
                         onPressed: {
                            j1_x_FWD_btn.color = "#dce1ec"
                            //发送点动信号1轴，正向
                            signal_StartJOG(1,1)
                        }
                        onReleased: {
                            j1_x_FWD_btn.color = "#3c4454"
                            //停止点动
                            signal_StopJOG()
                        }
                    }
                }

            }

            //J2轴，y轴
            RowLayout{
                id: j2_or_yAxis
                Layout.alignment: Qt.AlignHCenter
                spacing:25
                //减小按钮
                MyButtonText{
                    id: j2_y_RVS_btn
                    widthRect: 50
                    heightRect: 50
                    buttonText:'-'
                    pointSize:25
                    color:"#3c4454"  //初始设置为主页

                    // 鼠标悬浮时的背景色变化
                    MouseArea {

                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            j2_y_RVS_btn.color = "#dce1ec"
                            //信号测试
                            //robot_manual_layout.signal_test()
                            //发送点动信号1轴，反向
                            signal_StartJOG(2,-1)

                        }
                        onReleased: {
                            j2_y_RVS_btn.color = "#3c4454"
                            //停止点动
                            signal_StopJOG()
                        }
                        onClicked: {
                            console.log("clicked")
                        }
                    }
                }

                Text
                 {
                    text: "J2/YPos"  //
                    font.pointSize: 15
                    color: "#c3ccdf" // 字体颜色为白色
                }

                //增大按钮
                MyButtonText{
                    id: j2_y_FWD_btn
                    widthRect: 50
                    heightRect: 50
                    buttonText:'+'
                    pointSize:25
                    color:"#3c4454"  //初始设置为主页

                    // 鼠标悬浮时的背景色变化
                    MouseArea {

                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onEntered: {
                            // 鼠标进入区域时，按钮颜色变化
                            color = "#2c313c"
                        }
                         onPressed: {
                            j2_y_FWD_btn.color = "#dce1ec"
                            //发送点动信号1轴，正向
                            signal_StartJOG(2,1)
                        }
                        onReleased: {
                            j2_y_FWD_btn.color = "#3c4454"
                            //停止点动
                            signal_StopJOG()
                        }
                    }
                }

            }

            //J3轴，z轴
            RowLayout{
                id: j3_or_zAxis
                Layout.alignment: Qt.AlignHCenter
                spacing:25
                //减小按钮
                MyButtonText{
                    id: j3_z_RVS_btn
                    widthRect: 50
                    heightRect: 50
                    buttonText:'-'
                    pointSize:25
                    color:"#3c4454"  //初始设置为主页

                    // 鼠标悬浮时的背景色变化
                    MouseArea {

                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            j3_z_RVS_btn.color = "#dce1ec"
                            //信号测试
                            //robot_manual_layout.signal_test()
                            //发送点动信号1轴，反向
                            signal_StartJOG(3,-1)

                        }
                        onReleased: {
                            j3_z_RVS_btn.color = "#3c4454"
                            //停止点动
                            signal_StopJOG()
                        }
                        onClicked: {
                            console.log("clicked")
                        }
                    }
                }

                Text
                 {
                    text: "J3/ZPos"  //
                    font.pointSize: 15
                    color: "#c3ccdf" // 字体颜色为白色
                }

                //增大按钮
                MyButtonText{
                    id: j3_z_FWD_btn
                    widthRect: 50
                    heightRect: 50
                    buttonText:'+'
                    pointSize:25
                    color:"#3c4454"  //初始设置为主页

                    // 鼠标悬浮时的背景色变化
                    MouseArea {

                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onEntered: {
                            // 鼠标进入区域时，按钮颜色变化
                            color = "#2c313c"
                        }
                         onPressed: {
                            j3_z_FWD_btn.color = "#dce1ec"
                            //发送点动信号1轴，正向
                            signal_StartJOG(3,1)
                        }
                        onReleased: {
                            j3_z_FWD_btn.color = "#3c4454"
                            //停止点动
                            signal_StopJOG()
                        }
                    }
                }

            }

            //J4轴，x旋转轴
            RowLayout{
                id: j4_or_xRotAxis
                Layout.alignment: Qt.AlignHCenter
                spacing:25
                //减小按钮
                MyButtonText{
                    id: j4_xRot_RVS_btn
                    widthRect: 50
                    heightRect: 50
                    buttonText:'-'
                    pointSize:25
                    color:"#3c4454"  //初始设置为主页

                    // 鼠标悬浮时的背景色变化
                    MouseArea {

                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            j4_xRot_RVS_btn.color = "#dce1ec"
                            //信号测试
                            //robot_manual_layout.signal_test()
                            //发送点动信号1轴，反向
                            signal_StartJOG(4,-1)

                        }
                        onReleased: {
                            j4_xRot_RVS_btn.color = "#3c4454"
                            //停止点动
                            signal_StopJOG()
                        }
                        onClicked: {
                            console.log("clicked")
                        }
                    }
                }

                Text
                 {
                    text: "J4/XRot"  //
                    font.pointSize: 15
                    color: "#c3ccdf" // 字体颜色为白色
                }

                //增大按钮
                MyButtonText{
                    id: j4_xRot_FWD_btn
                    widthRect: 50
                    heightRect: 50
                    buttonText:'+'
                    pointSize:25
                    color:"#3c4454"  //初始设置为主页

                    // 鼠标悬浮时的背景色变化
                    MouseArea {

                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onEntered: {
                            // 鼠标进入区域时，按钮颜色变化
                            color = "#2c313c"
                        }
                         onPressed: {
                            j4_xRot_FWD_btn.color = "#dce1ec"
                            //发送点动信号1轴，正向
                            signal_StartJOG(4,1)
                        }
                        onReleased: {
                            j4_xRot_FWD_btn.color = "#3c4454"
                            //停止点动
                            signal_StopJOG()
                        }
                    }
                }

            }

            //J5轴，y旋转轴
            RowLayout{
                id: j5_or_yRotAxis  //JOG J1轴或x轴
                Layout.alignment: Qt.AlignHCenter
                spacing:25
                //减小按钮
                MyButtonText{
                    id: j5_yRot_RVS_btn
                    widthRect: 50
                    heightRect: 50
                    buttonText:'-'
                    pointSize:25
                    color:"#3c4454"  //初始设置为主页

                    // 鼠标悬浮时的背景色变化
                    MouseArea {

                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            j5_yRot_RVS_btn.color = "#dce1ec"
                            //信号测试
                            robot_manual_layout.signal_test()
                            //发送点动信号1轴，反向
                            signal_StartJOG(5,-1)

                        }
                        onReleased: {
                            j5_yRot_RVS_btn.color = "#3c4454"
                            //停止点动
                            signal_StopJOG()
                        }
                        onClicked: {
                            console.log("clicked")
                        }
                    }
                }

                Text
                 {
                    text: "J5/YRot"  //
                    font.pointSize: 15
                    color: "#c3ccdf" // 字体颜色为白色
                }

                //增大按钮
                MyButtonText{
                    id: j5_yRot_FWD_btn
                    widthRect: 50
                    heightRect: 50
                    buttonText:'+'
                    pointSize:25
                    color:"#3c4454"  //初始设置为主页

                    // 鼠标悬浮时的背景色变化
                    MouseArea {
                        id: mouseArea_j5_yRot_FWD_btn
                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onEntered: {
                            // 鼠标进入区域时，按钮颜色变化
                            color = "#2c313c"
                        }
                         onPressed: {
                            j5_yRot_FWD_btn.color = "#dce1ec"
                            //发送点动信号1轴，正向
                            signal_StartJOG(5,1)
                        }
                        onReleased: {
                            j5_yRot_FWD_btn.color = "#3c4454"
                            //停止点动
                            signal_StopJOG()
                        }
                    }
                }

            }

            //J6轴，z旋转轴
            RowLayout{
                id: j6_or_zRotAxis  //JOG J1轴或x轴
                Layout.alignment: Qt.AlignHCenter
                spacing:25
                //减小按钮
                MyButtonText{
                    id: j6_zRot_RVS_btn
                    widthRect: 50
                    heightRect: 50
                    buttonText:'-'
                    pointSize:25
                    color:"#3c4454"  //初始设置为主页

                    // 鼠标悬浮时的背景色变化
                    MouseArea {

                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            j6_zRot_RVS_btn.color = "#dce1ec"
                            //信号测试
                            robot_manual_layout.signal_test()
                            //发送点动信号1轴，反向
                            signal_StartJOG(6,-1)

                        }
                        onReleased: {
                            j6_zRot_RVS_btn.color = "#3c4454"
                            //停止点动
                            signal_StopJOG()
                        }
                        onClicked: {
                            console.log("clicked")
                        }
                    }
                }

                Text
                 {
                    text: "J6/ZRot"  //
                    font.pointSize: 15
                    color: "#c3ccdf" // 字体颜色为白色
                }

                //增大按钮
                MyButtonText{
                    id: j6_zRot_FWD_btn
                    widthRect: 50
                    heightRect: 50
                    buttonText:'+'
                    pointSize:25
                    color:"#3c4454"  //初始设置为主页

                    // 鼠标悬浮时的背景色变化
                    MouseArea {

                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onEntered: {
                            // 鼠标进入区域时，按钮颜色变化
                            color = "#2c313c"
                        }
                         onPressed: {
                            j6_zRot_FWD_btn.color = "#dce1ec"
                            //发送点动信号1轴，正向
                            signal_StartJOG(6,1)
                        }
                        onReleased: {
                            j6_zRot_FWD_btn.color = "#3c4454"
                            //停止点动
                            signal_StopJOG()
                        }
                    }
                }

            }
        }
    }

    //机械手臂状态信息更新
    function robot_int_state_update(value)
    {

    }

    //机械手臂状态信息更新
    function robot_float_state_update(value)
    {
        //通过工作空间选择显示的值
        if(robot_manual.workSpace_select==0)
        {
            _robot_j1_xPos_text = 'J1'
            _robot_j2_yPos_text = 'J2'
            _robot_j3_zPos_text = 'J3'
            _robot_j4_xRot_text = 'J4'
            _robot_j5_yRot_text = 'J5'
            _robot_j6_zRot_text = 'J6'

            _robot_j1_xPos_value=value[0]
            _robot_j2_yPos_value=value[1]
            _robot_j3_zPos_value=value[2]
            _robot_j4_xRot_value=value[3]
            _robot_j5_yRot_value=value[4]
            _robot_j6_zRot_value=value[5]
        }
        else
        {
            _robot_j1_xPos_text = 'xPos'
            _robot_j2_yPos_text = 'yPos'
            _robot_j3_zPos_text = 'zPos'
            _robot_j4_xRot_text = 'xRot'
            _robot_j5_yRot_text = 'yRot'
            _robot_j6_zRot_text = 'zRot'

            _robot_j1_xPos_value=value[6]
            _robot_j2_yPos_value=value[7]
            _robot_j3_zPos_value=value[8]
            _robot_j4_xRot_value=value[9]
            _robot_j5_yRot_value=value[10]
            _robot_j6_zRot_value=value[11]
        }
    }

    //工具坐标系名称加载到RobotManual
    function load_toolName_list(values)
    {
        console.log("load_tool_list")
        toolListMolde.clear()
        for(var val of values)
        {
            toolListMolde.append({name: val})
        }
        tool_comboBox.currentIndex = 0
    }

    //用户坐标系名称加载到RobotManual
    function load_userName_list(values)
    {
        console.log("load_user_list")
        userListMolde.clear()
        for(var val of values)
        {
            userListMolde.append({name: val})
        }
        user_comboBox.currentIndex = 0
    }

    //点位名称加载到RobotManual
    function load_pointsName_list(values)
    {
        console.log("load_point_list")
        robot_pointMolde.clear()
        for(var val of values)
        {
            robot_pointMolde.append({name: val})
        }
        point_comboBox.currentIndex = 0
    }

    //点位注释加载到RobotManual
    function load_pointsComment_list(values)
    {
        console.log("load_pointComment_list")
        point_comment_Molde.clear()
        for(var val of values)
        {
            point_comment_Molde.append({name: val})
        }
        point_comboBox.currentIndex = 0
        point_comment_current = point_comment_Molde.get(point_comboBox.currentIndex).name
    }

    //信号定义
    signal signal_test()
    //运动指令
    /*
    函数功能描述:机器人点动
    uint8_t ref - 0-关节点动, 2-基坐标系下点动, 4-工具坐标系下点动, 8-工件坐标系下点动
    uint8_t nb - 1-关节1(或x轴),2-关节2(或y轴),3-关节3(或z轴),4-关节4(或绕x轴旋转),5-关节5(或绕y轴旋转),6-关节6(或绕z轴旋转)
    uint8_t dir - 0-负方向, 1-正方向
    float vel - 速度百分比, 范围为0-100*/
    signal signal_StartJOG(int nb,int dir)

    /*
    函数功能描述:机器人点动停止
    uint8_t ref - 0-关节点动停止, 2-基坐标系下点动停止, 4-工具坐标系下点动停止, 8-工件坐标系下点动停止
    */
    signal signal_StopJOG()
    //workSpace改变
    signal signal_WorkSpaceChange(int ref_workSpace)
    //tool坐标系改变
    signal signal_toolChange(string tool_name)
    //用户坐标系改变
    signal signal_userChange(string user_name)
    //执行点位运动
    signal signal_pointMove(string pointName, string moveType,int exeFlag)
    //伺服上电信号
    signal signal_robotEnable(int server_status)

    // 定义属性来传递值
    property double _robot_j1_xPos_value: 123.000
    property double _robot_j2_yPos_value: 0.000
    property double _robot_j3_zPos_value: 0.000
    property double _robot_j4_xRot_value: 0.000
    property double _robot_j5_yRot_value: 0.000
    property double _robot_j6_zRot_value: 0.000

    property string _robot_j1_xPos_text: 'J1'
    property string _robot_j2_yPos_text: 'J2'
    property string _robot_j3_zPos_text: 'J3'
    property string _robot_j4_xRot_text: 'J4'
    property string _robot_j5_yRot_text: 'J5'
    property string _robot_j6_zRot_text: 'J6'

    property list<string> pointList:["p0"]

    property int workSpace_select
    property list<string> tool_list
    property list<string> user_list
    property list<string> point_comment_list
    property string point_comment_current: ""
    //伺服上电状态保存
    property bool serverOnOff_status: true

}