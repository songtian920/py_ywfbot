// 导入Qt Quick 2.15模块，提供构建用户界面的基本组件和功能
import QtQuick 2.15
import QtQuick.Controls 2.5
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15


Rectangle
{
    id: main_rect   //主背景框
    width: 1000 // 设置主背景框的宽度
    height: 650  // 设置主背景框的高度
    //height: 600
    color: "#2c313c"  // 设置主背景框的背景颜色
    radius: 10 // 设置圆角的半径大小

    Rectangle   //菜单框
    {
        id: menu_rect
        width: main_rect.width*0.13   //菜单框宽度
        height: main_rect.height-5   //菜单框高度
        anchors.left: parent.left //将标题锚定到父元素的左边
        //anchors.leftMargin: 5   // 设置左边距为5像素
        anchors.top: parent.top  //锚点 锚在父窗口上
        anchors.topMargin: 5   //锚点上 边距5像素
        color: "#1b1e23"
        radius: 10 // 设置圆角的半径大小

        Column{
            anchors.top: parent.top
            anchors.topMargin: 5
            anchors.left: parent.left
            anchors.leftMargin: 5
            anchors.right:parent.right
            anchors.rightMargin: 5
            spacing:10


            //菜单隐藏按钮
            MyButton
            {
                id: btn_hide_menu
                buttonText: "隐藏菜单" //
                width: parent.width
                height: main_rect.height*0.1
                color:"#21252d"  //初始设置为主页
                imageSourcePath:"svg_icons/icon_menu_close.svg"
                property bool visible_status:true
                // 重构鼠标事件
                MouseArea  //鼠标事件
                {
                    id: mouseArea_hide_menu
                    anchors.fill: parent

                    onClicked: {
                        // 按钮点击处理逻辑
                        if(btn_hide_menu.visible_status)
                        {
                            btn_hide_menu.visible_status=false  //菜单显示状态赋值真
                            btn_hide_menu.imageSourcePath="svg_icons/icon_menu.svg"
                            menu_rect.width=main_rect.width*0.05 //菜单框宽度
                            btn_hide_menu.visible_textBtn=false  //隐藏文字

                            btn_home_menu.visible_textBtn=false  //隐藏主页菜单文字
                            btn_manuel_menu.visible_textBtn=false  //隐藏手动菜单文字
                            btn_dataSet_menu.visible_textBtn=false //数据设置隐藏文字
                            btn_processConfig_menu.visible_textBtn=false  //隐藏参数配置菜单文字
                            btn_login_menu.visible_textBtn=false
                            btn_information_menu.visible_textBtn=false
                            console.log("隐藏主页")
                        }
                        else
                        {
                            btn_hide_menu.visible_status=true  //菜单显示状态赋值假
                            btn_hide_menu.imageSourcePath="svg_icons/icon_menu_close.svg"

                            menu_rect.width=main_rect.width*0.15   //菜单框宽度
                            btn_hide_menu.visible_textBtn=true  //显示隐藏菜单文字

                            btn_home_menu.visible_textBtn=true  //显示主页菜单文字
                            btn_manuel_menu.visible_textBtn=true  //显示手动菜单文字
                            btn_dataSet_menu.visible_textBtn=true //数据设置隐藏文字
                            btn_processConfig_menu.visible_textBtn=true  //显示参数配置菜单文字
                            btn_login_menu.visible_textBtn=true
                            btn_information_menu.visible_textBtn=true
                            console.log("显示主页")
                        }
                    }
                }
            }

            MyButton  //主页按钮
            {
                id: btn_home_menu
                width: parent.width
                height: menu_rect.height*0.1
                buttonText: "主页" // 设置按钮文本为 "1"
                color:"#2c313c"  //初始设置为主页
                imageSourcePath:"svg_icons/icon_home.svg"
                // 鼠标悬浮时的背景色变化
                MouseArea  //鼠标事件
                {
                    id: mouseArea_home
                    anchors.fill: parent
                    onClicked: {
                        // 按钮点击处理逻辑
                        btn_home_menu.color="#2c313c"
                        btn_manuel_menu.color="#1b1e23"
                        btn_dataSet_menu.color="#1b1e23"
                        btn_processConfig_menu.color="#1b1e23"
                        btn_login_menu.color="#1b1e23"
                        //console.log("主页")
                        stackLayout_win.currentIndex=0
                        //界面监控IO关闭
                        signal_IoSetSwitch(false)
                    }
                }
            }

            //手动按钮
            MyButton
            {
                id: btn_manuel_menu
                width: parent.width
                height: menu_rect.height*0.1
                buttonText: "手动" // 设置按钮文本为 "1"
                imageSourcePath:"svg_icons/icon_widgets.svg"
                MouseArea   //鼠标事件
                {
                    id: mouseArea_btn_manuel_menu
                    anchors.fill: parent
                    onClicked: {
                        // 按钮点击处理逻辑
                        btn_home_menu.color="#1b1e23"
                        btn_manuel_menu.color="#2c313c"
                        btn_dataSet_menu.color="#1b1e23"
                        btn_processConfig_menu.color="#1b1e23"
                        btn_login_menu.color="#1b1e23"
                        stackLayout_win.currentIndex=1
                        //console.log("手动")
                        //界面监控IO开始
                        signal_IoSetSwitch(true)
                    }
                }
            }

            MyButton  //数据设置
            {
                id: btn_dataSet_menu
                width: parent.width
                height: menu_rect.height*0.1

                buttonText: "数据设置" // 设置按钮文本为 "1"
                imageSourcePath:"svg_icons/icon_file.svg"
                MouseArea   //鼠标事件
                {
                    id: mouseArea_dataSet
                    anchors.fill: parent
                    onClicked: {
                        // 按钮点击处理逻辑
                        btn_home_menu.color="#1b1e23"
                        btn_manuel_menu.color="#1b1e23"
                        btn_dataSet_menu.color="#2c313c"
                        btn_processConfig_menu.color="#1b1e23"
                        btn_login_menu.color="#1b1e23"
                        stackLayout_win.currentIndex=2
                        //console.log("配置参数")
                        //界面监控IO关闭
                        signal_IoSetSwitch(false)
                    }
                }
            }


            MyButton  //配置按钮
            {
                id: btn_processConfig_menu
                width: parent.width
                height: menu_rect.height*0.1
                buttonText: "工艺参数" // 设置按钮文本为 "1"
                imageSourcePath:"svg_icons/icon_settings.svg"
                MouseArea   //鼠标事件
                {
                    id: mouseArea_config
                    anchors.fill: parent
                    onClicked: {
                        // 按钮点击处理逻辑
                        btn_home_menu.color="#1b1e23"
                        btn_manuel_menu.color="#1b1e23"
                        btn_dataSet_menu.color="#1b1e23"
                        btn_processConfig_menu.color="#2c313c"
                        btn_login_menu.color="#1b1e23"
                        stackLayout_win.currentIndex=3
                        //console.log("配置参数")
                        //界面监控IO关闭
                        signal_IoSetSwitch(false)
                    }
                }
            }

            //用户权限按钮
            MyButton
            {
                id: btn_login_menu
                width: parent.width
                height: menu_rect.height*0.1
                buttonText: "用户权限" // 设置按钮文本为 "1"
                imageSourcePath:"svg_icons/icon_add_user.svg"
                MouseArea   //鼠标事件
                {
                    id: mouseArea_login
                    anchors.fill: parent
                    onClicked: {
                        // 按钮点击处理逻辑
                        btn_home_menu.color="#1b1e23"
                        btn_manuel_menu.color="#1b1e23"
                        btn_dataSet_menu.color="#1b1e23"
                        btn_processConfig_menu.color="#1b1e23"
                        btn_login_menu.color="#2c313c"
                        stackLayout_win.currentIndex=4
                        //console.log("配置参数")
                        //界面监控IO关闭
                        signal_IoSetSwitch(false)
                    }
                }
            }
        }

        Canvas {
            id: lineCanvas
            anchors.fill: parent
            onPaint: {
                var ctx = getContext("2d");
                ctx.strokeStyle = "#636c7b";
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(10, main_rect.height*0.85); // 起点坐标 (x, y)
                ctx.lineTo(width-10, main_rect.height*0.85); // 终点坐标 (x, y)
                ctx.stroke(); // 绘制直线
            }
        }

        //信息按钮
        MyButton
        {
            id: btn_information_menu
            anchors.left: menu_rect.left  //左边界
            anchors.leftMargin: 5
            anchors.right: menu_rect.right
            anchors.rightMargin: 5
            anchors.bottom: menu_rect.bottom
            anchors.bottomMargin: 20
            color:"#1b1e23"
            height: menu_rect.height*0.06
            buttonText: "信息" // 设置按钮文本为 "1"
            imageSourcePath:"svg_icons/icon_add_user.svg"
            property bool infor_menu_visible_status: false
            MouseArea   //鼠标事件
            {
                id: mouseArea_information_menu
                anchors.fill: parent
                onClicked: {
                    // 按钮点击处理逻辑
                    if(info_panel.visible)
                    {
                        btn_information_menu.color="#1b1e23"
                        info_panel.visible=false
                        //console.log("配置参数")
                    }
                    else
                    {
                        btn_information_menu.color="#2c313c"
                        info_panel.visible=true
                    }

                }
            }
        }

    }

    //工具状态框
    Rectangle
    {
        id: tool_rect
        anchors.top: parent.top  // 将标题锚定到父元素的顶部
        anchors.topMargin: 5  // 设置顶部边距为5像素
        anchors.left: menu_rect.right  //工具左 对齐菜单右
        anchors.leftMargin: 5  //间距5
        anchors.right: parent.right
        anchors.rightMargin: 5
        //width: parent.width*0.85-10
        height: parent.height*0.07
        color: "#343b48"
        radius: 10 // 设置圆角的半径大小

        //功能窗口按钮
        MyButtonIcon
        {
            id: btn_frRobot_panel
            widthRect: 35
            heightRect: parent.height*0.8
            z:90
            anchors.verticalCenter: parent.verticalCenter // 设置按钮垂直居中
            anchors.right: tool_rect.right
            anchors.rightMargin: 180

            color:"#3c4454"  //初始设置为主页
            imageSourcePath:"svg_icons/icon_settings.svg"
            // 鼠标悬浮时的背景色变化
            MouseArea {
                id: mouseArea_btn_function_panel
                anchors.fill: parent
                hoverEnabled: true // 设置为true以启用悬停事件
                onEntered: {
                    // 鼠标进入区域时，按钮颜色变化
                    btn_frRobot_panel.color = "#2c313c"
                    console.log("mouseIn")
                }
                onExited: {
                    // 鼠标进入区域时，按钮颜色变化
                    btn_frRobot_panel.color = "#3c4454"
                }

                onClicked: {
                    // 按钮点击处理逻辑
                    if(frRobot_panel.visible)
                    {
                        btn_frRobot_panel.color="#2c313c"
                        frRobot_panel.visible=false
                        signal_robotManuelSwitch(false)

                    }
                    else
                    {
                        btn_frRobot_panel.color="#1b1e23"
                        frRobot_panel.visible=true
                        signal_robotManuelSwitch(true)
                    }

                }
            }
        }

        Canvas {
            id: lineCanvas_tool_rect
            anchors.fill: parent
            onPaint: {
                var ctx = getContext("2d");
                ctx.strokeStyle = "#636c7b";
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(tool_rect.width-150, 0); // 起点坐标 (x, y)
                ctx.lineTo(tool_rect.width-150, tool_rect.height); // 终点坐标 (x, y)
                ctx.stroke(); // 绘制直线
            }
        }

        //窗口控制按钮
        RowLayout
        {
            id: win_control_RowLayout
            //width: parent.width*0.4
            height: parent.height
            width: 120

            anchors.right: parent.right
            anchors.rightMargin: 5
            anchors.top: parent.top
            anchors.bottom: parent.bottom

            spacing: 2 // 设置子元素之间的间距为10个像素

            //窗口最小按钮
            MyButtonIcon
            {
                id: win_min
                widthRect: 35
                heightRect: parent.height*0.8
                Layout.alignment: Qt.AlignVCenter // 设置按钮垂直居中

                color:"#3c4454"  //初始设置为主页
                imageSourcePath:"svg_icons/icon_minimize.svg"
                // 鼠标悬浮时的背景色变化
                MouseArea {
                    id: mouseArea_win_min
                    anchors.fill: parent
                    hoverEnabled: true // 设置为true以启用悬停事件
                    onEntered: {
                        // 鼠标进入区域时，按钮颜色变化
                        win_min.color = "#2c313c"
                        console.log("mouseIn")
                    }
                    onExited: {
                        // 鼠标进入区域时，按钮颜色变化
                        win_min.color = "#3c4454"
                    }
                    onClicked: {
                        // 按钮点击处理逻辑
                        console.log("clicked")
                        //info_panel.visible=false
                    }
                }
            }

            //窗口最大按钮
            MyButtonIcon
            {
                id: win_max
                widthRect: 35
                heightRect: parent.height*0.8
                Layout.alignment: Qt.AlignVCenter // 设置按钮垂直居中

                color:"#3c4454"  //初始设置为主页
                imageSourcePath:"svg_icons/icon_maximize.svg"
                // 鼠标悬浮时的背景色变化
                MouseArea {
                    id: mouseArea_win_max
                    anchors.fill: parent
                    hoverEnabled: true // 设置为true以启用悬停事件
                    onEntered: {
                        // 鼠标进入区域时，按钮颜色变化
                        win_max.color = "#2c313c"
                    }
                    onExited: {
                        // 鼠标进入区域时，按钮颜色变化
                        win_max.color = "#3c4454"
                    }
                    onClicked: {
                        // 按钮点击处理逻辑
                        //info_panel.visible=false
                        console.log("clicked")
                    }
                }
            }

            //窗口关闭按钮
            MyButtonIcon
            {
                id: win_shutup
                widthRect: 35
                heightRect: parent.height*0.8
                Layout.alignment: Qt.AlignVCenter // 设置按钮垂直居中

                color:"#3c4454"  //初始设置为主页
                imageSourcePath:"svg_icons/icon_close.svg"

                // 鼠标悬浮时的背景色变化
                MouseArea {
                    id: mouseArea_win_shutup
                    anchors.fill: parent
                    hoverEnabled: true // 设置为true以启用悬停事件
                    onEntered: {
                        // 鼠标进入区域时，按钮颜色变化
                        win_shutup.color = "#2c313c"
                    }
                    onExited: {
                        // 鼠标进入区域时，按钮颜色变化
                        win_shutup.color = "#3c4454"
                    }
                    onClicked: {
                        // 按钮点击处理逻辑
                        //info_panel.visible=false
                        console.log("clicked")
                    }
                }
            }
        }

        //状态显示
        //app状态显示
        Text {
            id: text_appStatus
            anchors.top: parent.top
            //anchors.topMargin: 5
            anchors.bottom: parent.bottom
            //anchors.bottomMargin: 5
            anchors.left: parent.left
            anchors.leftMargin: 20
            text: '未初始化完成' // 绑定到外部传入的属性
            font.pointSize: 20
            color: "red" // 字体颜色为红色
            verticalAlignment: Text.AlignVCenter
            Layout.fillWidth: true
        }
    }

    //主窗体矩形框
    Rectangle{
        id: rect_mainContainer
        anchors.top: tool_rect.bottom
        anchors.topMargin: 5
        anchors.bottom: main_rect.bottom
        anchors.bottomMargin: 50
        anchors.left: menu_rect.right
        anchors.leftMargin: 5
        anchors.right: main_rect.right
        anchors.rightMargin: 5
        color: "#2c313c"
        radius: 10

        //信息面板
        Rectangle{
            id: info_panel
            height: rect_mainContainer.height
            width: rect_mainContainer.width/4
            anchors.left: parent.left
            z:90
            //width: 200
            //height: 200
            Layout.alignment:  Qt.AlignVCenter
            radius: 10  //圆角半径
            color: "#343b48"
            //color: "blue"
            visible: false

            //标题框
            Rectangle{
                id: info_panel_title
                //width: parent.width*0.8
                height: parent.height*0.05
                anchors.top: parent.top
                anchors.topMargin: 10
                anchors.left: parent.left
                anchors.leftMargin: 5
                anchors.right: parent.right
                anchors.rightMargin: 5
                radius:10
                color: "#3c4454"
                // 文字部分
                Text {
                    text: "info tab" // 绑定到外部传入的属性
                    font.pointSize: 10
                    color: "#c3ccdf" // 字体颜色为白色
                    // 设置锚点使其垂直居中
                    anchors.left: parent.left
                    anchors.leftMargin: 15
                    anchors.verticalCenter: parent.verticalCenter
                    Layout.fillWidth: true
                }
            }
        }

        //抽屉面板
        StackLayout
        {
            id: stackLayout_win
            anchors.fill: parent
            visible: true
            currentIndex: 0

            //主页窗体
            Rectangle
            {
                id: win_home_rect
                color: "#3c4454"
                //color: "blue"
                radius: 10
                width: parent.width
                height: parent.height

                ColumnLayout{
                    id: btn_layout
                    anchors.left: parent.left
                    anchors.leftMargin: 10
                    anchors.top: parent.top
                    anchors.topMargin: 10
                    width: parent.width*0.2
                    height: parent.height*0.6

                    //自动手动切换开关
                    MyButtonText{
                        id: switch_autoManuel
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        buttonText: "手动"
                        color:"#e58364"

                        // 鼠标悬浮时的背景色变化
                        MouseArea {
                            id: switch_autoManuel_mouseArea
                            anchors.fill: parent
                            hoverEnabled: true // 设置为true以启用悬停事件
                            onPressed: {
                                switch_autoManuel.color = "#dce1ec"
                            }
                            onClicked: {
                                // 按钮点击处理逻辑
                                if(switch_autoManuel.buttonStatus)
                                {
                                    //按钮状态真，切换为手动
                                    //switch_autoManuel.color="#e58364"
                                    //switch_autoManuel.buttonText="手动"
                                    //switch_autoManuel.buttonStatus=false
                                    //发送手自动切换信号
                                    signal_manuel_auto_switch(false)
                                }
                                else
                                {
                                    //切换自动
                                    //switch_autoManuel.color="#568af2"
                                    //switch_autoManuel.buttonText="自动"
                                    //switch_autoManuel.buttonStatus=true
                                    signal_manuel_auto_switch(true)
                                }
                            }
                        }
                    }

                    //启动
                    MyButtonText{
                        id: btn_start
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        buttonText: "启动"
                        color: "#21252d" // 默认背景

                        // 鼠标悬浮时的背景色变化
                        MouseArea {
                            id: btn_start_mouseArea
                            anchors.fill: parent
                            hoverEnabled: true // 设置为true以启用悬停事件
                            onPressed: {
                                btn_start.color = "#dce1ec"
                            }
                            onClicked: {
                                // 按钮点击处理逻辑
                                //发射启动信号
                                signal_start_pause_switch(true)
                            }
                        }
                    }

                    //停止
                    MyButtonText{
                        id: btn_stop
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        buttonText: "暂停"
                        color: "#21252d" // 默认背景

                        // 鼠标悬浮时的背景色变化
                        MouseArea {
                            id: btn_stop_mouseArea
                            anchors.fill: parent
                            hoverEnabled: true // 设置为true以启用悬停事件

                            onPressed: {
                                btn_stop.color = "#dce1ec"
                            }
                            onReleased: {
                                btn_stop.color = "#21252d"


                            }
                            onClicked: {
                                //发射暂停信号
                                signal_start_pause_switch(false)

                            }
                        }
                    }

                    //清除报警
                    MyButtonText{
                        id: btn_clearAlarm
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        buttonText: "清除报警"
                        color: "#21252d" // 默认背景

                        // 鼠标悬浮时的背景色变化
                        MouseArea {
                            id: btn_clearAlarm_mouseArea
                            anchors.fill: parent
                            hoverEnabled: true // 设置为true以启用悬停事件

                            onPressed: {
                                btn_clearAlarm.color = "#dce1ec"
                            }
                            onReleased: {
                                btn_clearAlarm.color = "#21252d"

                            }
                            onClicked: {
                                signal_clear_errorList()
                            }
                        }
                    }
                }

                //机器人状态信息显示
                AmrStatusDisp{
                    id: amrStatusDisp
                    anchors.top: parent.top
                    anchors.topMargin: 15
                    anchors.right: parent.right
                    anchors.rightMargin: 15
                }
            }

            //manual_win窗体
            Rectangle
            {
                id: manualControl_win
                color: "#3c4454"
                //color: "blue"
                radius: 10
                width: parent.width
                height: parent.height

                //选择按钮区
                Row{
                    id: select_btn_row
                    anchors.top: parent.top
                    anchors.topMargin:10
                    anchors.left:parent.left
                    anchors.leftMargin:100
                    spacing:60
                    //io控制 手臂io控制
                    MyButton
                    {
                        id: robotIo_btn
                        width: 150
                        height: 50
                        color: "#2c313c"
                        buttonText: "手臂IO点"
                        imageSourcePath:"svg_icons/robot.jpg"
                        MouseArea   //鼠标事件
                        {
                            anchors.fill: parent
                            onClicked: {
                                // 按钮点击处理逻辑
                                robotIo_btn.color = "#2c313c"
                                amrIo_btn.color = "#1b1e23"
                                manualControl_StackLayout.currentIndex=0
                            }
                        }
                    }
                    //amrIO控制
                    MyButton
                    {
                        id:amrIo_btn
                        width: 150
                        height: 50

                        buttonText: "车体IO点" // 设置按钮文本为 "1"
                        imageSourcePath:"svg_icons/robot.jpg"
                        MouseArea   //鼠标事件
                        {
                            anchors.fill: parent
                            onClicked: {
                                // 按钮点击处理逻辑
                                robotIo_btn.color = "#1b1e23"
                                amrIo_btn.color = "#2c313c"
                            }
                        }
                    }
                }
                //抽屉布局
                StackLayout
                {
                    id: manualControl_StackLayout
                    anchors.top: select_btn_row.bottom
                    anchors.topMargin: 10
                    anchors.bottom: parent.bottom
                    anchors.left: parent.left
                    anchors.leftMargin: 5
                    anchors.right: parent.right
                    anchors.rightMargin: 5
                    visible: true
                    currentIndex: 0

                    //robot io窗口
                    RobotIoSet{
                        id: robotIoSet_win
                        //anchors.fill: parent
                    }
                }
            }

            //数据设置窗体
            Rectangle {
                id: win_dataSet
                color: "#3c4454"
                radius: 10
                width: parent.width
                height: parent.height
                //选择控制按钮
                Row{
                    id: row_btn_dataSet
                    anchors.top: parent.top
                    anchors.topMargin:10
                    anchors.left:parent.left
                    anchors.leftMargin:100
                    spacing:60
                    //机器人点位表切换
                    MyButton
                    {
                        id: btn_robotPoint_rect
                        width: 150
                        height: 50
                        color: "#2c313c"
                        buttonText: "手臂点位" // 设置按钮文本为 "1"
                        imageSourcePath:"svg_icons/robot.jpg"
                        MouseArea   //鼠标事件
                        {

                            anchors.fill: parent
                            onClicked: {
                                // 按钮点击处理逻辑
                                btn_robotPoint_rect.color="#2c313c"
                                btn_robotTool_rect.color="#1b1e23"
                                btn_robotUser_rect.color="#1b1e23"
                                //btn_cameraParam_rect.color="#1b1e23"
                                dataSet_StackLayout.currentIndex=0

                            }
                        }
                    }
                    //手臂工具坐标系
                    MyButton
                    {
                        id:btn_robotTool_rect
                        width: 150
                        height: 50

                        buttonText: "手臂工具坐标系" // 设置按钮文本为 "1"
                        imageSourcePath:"svg_icons/robot.jpg"
                        MouseArea   //鼠标事件
                        {

                            anchors.fill: parent
                            onClicked: {
                                // 按钮点击处理逻辑
                                btn_robotTool_rect.color="#2c313c"
                                btn_robotPoint_rect.color="#1b1e23"
                                btn_robotUser_rect.color="#1b1e23"
                                //btn_cameraParam_rect.color="#1b1e23"
                                dataSet_StackLayout.currentIndex=1
                            }
                        }
                    }
                    //手臂用户坐标系
                    MyButton
                    {
                        id:btn_robotUser_rect
                        width: 150
                        height: 50

                        buttonText: "手臂用户坐标系" // 设置按钮文本为 "1"
                        imageSourcePath:"svg_icons/robot.jpg"
                        MouseArea   //鼠标事件
                        {
                            anchors.fill: parent
                            onClicked: {
                                // 按钮点击处理逻辑
                                btn_robotTool_rect.color="#1b1e23"
                                btn_robotPoint_rect.color="#1b1e23"
                                btn_robotUser_rect.color="#2c313c"
                                //btn_cameraParam_rect.color="#1b1e23"
                                dataSet_StackLayout.currentIndex=2
                            }
                        }
                    }

                }

                //抽屉表格
                StackLayout
                {
                    id: dataSet_StackLayout
                    anchors.top: row_btn_dataSet.bottom
                    anchors.topMargin: 10
                    anchors.bottom: parent.bottom
                    anchors.left: parent.left
                    anchors.leftMargin: 5
                    anchors.right: parent.right
                    anchors.rightMargin: 5
                    visible: true
                    currentIndex: 0

                    //手臂点表设置
                    RobotPointsTable{
                        id:robotPointTable
                    }

                    //手臂工具表设置
                    RobotToolTable{
                        id:robotToolTable
                    }

                    //手臂工具表设置
                    RobotUserTable{
                        id:robotUserTable
                    }

                }
            }

            //工艺参数窗体
            Rectangle {
                id: win_processConfig
                color: "#3c4454"
                radius: 10
                width: parent.width
                height: parent.height
                property list<string> button_process_names:["码垛工艺","堆叠工艺","打磨工艺","视觉引导抓取"]
                Grid {
                    anchors.centerIn: parent
                    columns: 3 // 设置 3 列
                    spacing: 50 // 间距 5px
                    //子元素自动按网格排列
                    Repeater {
                        model: 4 // 创建 9 个子项
                        //信息按钮
                        MyButton
                        {
                            id: process_select_btn
                            color:"#1b1e23"
                            width: 150
                            height: 60
                            buttonText: win_processConfig.button_process_names[index] // 设置按钮文本为 "1"
                            imageSourcePath:"svg_icons/processButton"+index+".jpg"
                            MouseArea   //鼠标事件
                            {
                                anchors.fill: parent
                                onPressed: {
                                    process_select_btn.color = "#dce1ec"
                                }
                                onReleased: {
                                    process_select_btn.color = "#1b1e23"
                                }
                                onClicked: {
                                    if(index==0)   //打开码垛工艺包
                                    {
                                        palletProcess.visible=true
                                    }
                                    else if(index==1)
                                    {

                                    }
                                    else if(index==2)
                                    {

                                    }
                                    else if(index==2)
                                    {

                                    }
                                }
                            }
                        }
                    }
                }
                //码垛工艺包窗口
                PalletizingProcess{
                    id: palletProcess
                    visible:false
                    anchors.fill: parent
                }

            }

            //用户权限窗体
            Rectangle {
                id: win_userLimits
                color: "#2c313c"
                radius: 10
                width: parent.width
                height: parent.height
            }
        }

        //功能面板
        Rectangle
        {
            id: frRobot_panel
            height: rect_mainContainer.height
            width: rect_mainContainer.width*4/5
            radius: 10  //圆角半径
            anchors.right: parent.right
            color: "#343b48"
            visible: false

            //机器人手动
            RobotManual{
                id: robot_manual
                anchors.centerIn: parent

            }
        }
    }

    //报警信息及其他信息
    Rectangle{
        id: rect_alarmMesAndStatus
        height: parent.height*0.06
        anchors.left: menu_rect.right
        anchors.leftMargin: 5
        anchors.right: parent.right
        anchors.rightMargin: 5
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 5
        radius: 10
        color: "#343b48"
        z:90

        MyButtonText{
            id: alarm_disp_btn
            anchors.top: parent.top
            anchors.topMargin:3
            anchors.right: parent.right
            anchors.rightMargin:5

            width: 40
            height: 35
            color: "#c3ccdf" // 字体颜色为白色
            z:90

            Text{
                id:text_alarm_disp_btn
                text: "展开"
                font.pointSize: 7
                anchors.centerIn: parent

            }

            MouseArea {
                id: alarm_disp_btn_mouseArea
                anchors.fill: parent
                hoverEnabled: true // 设置为true以启用悬停事件
                onClicked: {
                    if(text_alarm_disp_btn.text=="展开")
                    {
                        rect_alarmMesAndStatus.height= main_rect.height*0.3
                        text_alarm_disp_btn.text="折叠"
                        console.log("展开")
                    }
                    else
                    {
                        rect_alarmMesAndStatus.height= main_rect.height*0.06
                        text_alarm_disp_btn.text="展开"
                        console.log("折叠")
                    }
                }
            }
        }

        //报警显示
        ListView {
            id: errorListView
              //anchors.fill: parent
            anchors.left: parent.left
            anchors.leftMargin:50
            anchors.top: parent.top
            anchors.topMargin:10
            anchors.right:parent.right
            anchors.rightMargin:70
            anchors.bottom: parent.bottom
            anchors.bottomMargin:10
            z:90

            model: ListModel {
                id: errorLogModel
                ListElement {
                    errLogText: "异常信息显示"
                }

                // 可以根据需要添加更多日志
            }
            focus:true
            clip:true
            //委托
            delegate: errItemComponent
            //委托组件
            Component{
                id:errItemComponent
                Rectangle{
                    id:errItemRect
                    width: ListView.view.width;height: 25

                    color:ListView.isCurrentItem? "red":"#e56248"
                    Text {
                        verticalAlignment: Text.AlignVCenter
                        text: errLogText
                        color: "white"
                        font.pointSize: 12
                    }
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            errorListView.currentIndex = index
                        }
                    }

                }
            }
            spacing: 5
            ScrollBar.vertical: ScrollBar {
            }
        }
    }

    // 定义组件完成后的处理函数 信号绑定 槽函数
    Component.onCompleted: function() {
        //python给QML发信号 python信号绑定qml槽函数

        //初始化成功 反馈给界面信号 绑定槽函数
        signal_obj.init_success_signal.connect(init_success_signal_func);
        //手动自动切换开关成功python 反馈信号 绑定槽函数
        signal_obj.manuel_auto_signal.connect(manuel_auto_signal_func)
        //启动暂停 切换 反馈信号 绑定槽函数
        signal_obj.start_status_signal.connect(start_pause_signal_fun)
        //报警信息 python反馈信号 绑定槽函数
        signal_obj.signal_errorMesAppend.connect(addErrorLog)
        //报警信息 清除 python反馈信号 绑定槽函数
        signal_obj.signal_errorMesClear.connect(clearErrorLog)

        //python给QML发信号 python信号绑定qml槽函数
        //RobotManual
        //手臂状态更新
        signal_obj.signal_robot_int_state_update.connect(robot_manual.robot_int_state_update)
        signal_obj.signal_robot_float_state_update.connect(robot_manual.robot_float_state_update)
        signal_obj.signal_robot_tool_list.connect(robot_manual.load_toolName_list)
        signal_obj.signal_robot_user_list.connect(robot_manual.load_userName_list)
        signal_obj.signal_robot_pointsName_list.connect(robot_manual.load_pointsName_list)
        signal_obj.signal_robot_pointsComment_list.connect(robot_manual.load_pointsComment_list)

        //AmrStatusDisp
        signal_obj.signal_robotSpeed_callback.connect(amrStatusDisp.robotSpeed_callback)

        //手臂点表信号添加
        //手臂点表 添加数据
        signal_obj.signal_appendRow_tableModel_frPoint.connect(robotPointTable.appendRow_tableModel)
        signal_obj.signal_insertNewRowToTM_DB_frPoint.connect(robotPointTable.insertNewRowToTM_DB)
        //示教点位数据回传信号
        signal_obj.signal_teach_frPointsList.connect(robotPointTable.teach_point_callback)

        //手臂工具坐标表
        signal_obj.signal_appendRow_tableModel_frTool.connect(robotToolTable.appendRow_tableModel)
        signal_obj.signal_insertNewRowToTM_DB_frTool.connect(robotToolTable.insertNewRowToTM_DB)
        //新工具示教点位 返回数据
        signal_obj.signal_teachPoint_calibTool_callBack.connect(robotToolTable.teach_point_callBack_toolTable)
        //新工具计算 返回数据
        signal_obj.signal_calculateTool_calibTool_callback.connect(robotToolTable.calculate_tool_callback_toolTable)
        //新工具结果回传到 界面显示
        signal_obj.signal_updateToolData_calibTool_callback.connect(robotToolTable.updateToolData_calibTool_callback)

        //手臂用户坐标表
        signal_obj.signal_appendRow_tableModel_frUser.connect(robotUserTable.appendRow_tableModel)
        signal_obj.signal_insertNewRowToTM_DB_frUser.connect(robotUserTable.insertNewRowToTM_DB)
        //新用户示教点位 返回数据
        signal_obj.signal_teachPoint_calibUser_callBack.connect(robotUserTable.teach_point_callBack_userTable)
        //新用户计算结果 回传到界面
        signal_obj.signal_calculateUser_calibUser_callback.connect(robotUserTable.calculate_user_callback_userTable)
        //新用户结果回传到 界面显示
        signal_obj.signal_updateUserData_calibUser_callback.connect(robotUserTable.updateUserData_calibUser_callback)

        //robot manual
        signal_obj.signal_update_IO_state_robotDIO.connect(robotIoSet_win.update_IO_state)

        //码垛工艺包 回馈信号
        signal_obj.signal_insert_newPalletProcess_callback.connect(palletProcess.insertNewParamPalletProcessToTM)
        signal_obj.signal_appendParam_palletProcess_callback.connect(palletProcess.appendParamPalletProcess)
        signal_obj.signal_sendToolUserPointNames_callback.connect(palletProcess.getToolUserPointNames_callback)

        //robot_manual信号绑定到主信号
        robot_manual.signal_test.connect(mySignal)  //测试
        robot_manual.signal_StartJOG.connect(signal_StartJOG_main)  //点动开始信号
        robot_manual.signal_StopJOG.connect(signal_StopJOG_main)    //点动停止信号
        robot_manual.signal_WorkSpaceChange.connect(signal_WorkSpaceChange_main)  //工作空间改变信号
        robot_manual.signal_toolChange.connect(signal_toolChange_main)  //工具坐标系改变
        robot_manual.signal_userChange.connect(signal_userChange_main)  //用户坐标系
        robot_manual.signal_pointMove.connect(signal_pointMove_main)  //点动运动坐标
        robot_manual.signal_robotEnable.connect(signal_robotEnable_main)  //伺服上电断电信号

        //AmrStatusDisp信号绑定到主信号
        amrStatusDisp.signal_robotSpeed.connect(signal_robotSpeed_main)   //手臂速度值改变信号绑定


        //robot 点表信号绑定到主信号
        robotPointTable.signal_deleteDbRow_robotPointList.connect(signal_deleteDbRow_robotPointList_main)
        robotPointTable.signal_updateDbRow_robotPointList.connect(signal_updateDbRow_robotPointList_main)
        robotPointTable.signal_insert_row_robotPointList.connect(signal_insert_row_robotPointList_main)
        robotPointTable.signal_refresh_DBPointToTableView.connect(signal_refresh_DBPointToTableView_main)
        robotPointTable.signal_point_teach.connect(signal_point_teach_main)
        robotPointTable.signal_execute_point.connect(signal_execute_point_main)

        //robot 工具坐标系信号绑定主信号
        robotToolTable.signal_deleteDbRow_robotToolList.connect(signal_deleteDbRow_robotToolList_main)
        robotToolTable.signal_updateDbRow_robotToolList.connect(signal_updateDbRow_robotToolList_main)
        robotToolTable.signal_insert_row_robotToolList.connect(signal_insert_row_robotToolList_main)
        robotToolTable.signal_refresh_DBToolToTableView.connect(signal_refresh_DBToolToTableView_main)
        //新工具点位示教
        robotToolTable.signal_teach_tool_point_toolTable.connect(signal_teach_tool_point_main)
        //计算新工具平移变换信号绑定
        robotToolTable.signal_calculate_tool_trans_toolTable.connect(signal_calculate_tool_trans_main)
        //将新工具平移变换 更新到界面及数据库信号绑定
        robotToolTable.signal_update_tool_trans_toolTable.connect(signal_update_tool_trans_main)
        //计算工具旋转变换
        robotToolTable.signal_calculate_tool_rotate_toolTable.connect(signal_calculate_tool_rotate_main)
        //更新工具旋转变换
        robotToolTable.signal_update_tool_rotate_toolTable.connect(signal_update_tool_rotate_main)

        //robot 用户坐标系信号绑定到主信号
        robotUserTable.signal_deleteDbRow_robotUserList.connect(signal_deleteDbRow_robotUserList_main)
        robotUserTable.signal_updateDbRow_robotUserList.connect(signal_updateDbRow_robotUserList_main)
        robotUserTable.signal_insert_row_robotUserList.connect(signal_insert_row_robotUserList_main)
        robotUserTable.signal_refresh_DBUserToTableView.connect(signal_refresh_DBUserToTableView_main)
        //新用户点位示教 信号绑定
        robotUserTable.signal_teach_user_point_userTable.connect(signal_teach_user_point_userTable_main)
        //计算新用户位姿变换
        robotUserTable.signal_calculate_user_coord_userTable.connect(signal_calculate_user_coord_userTable_main)
        //更新新用户在基坐标系位姿
        robotUserTable.signal_update_user_coord_userTable.connect(signal_update_user_coord_userTable_main)

        //robotDIOSet手臂IO
        //robot SetDo
        robotIoSet_win.signal_SetDO.connect(signal_SetDO_RobotIoSet_main)
        //robot tag更新
        robotIoSet_win.signal_update_RobotDIO_tag.connect(signal_update_RobotDIO_tag_main)

        //码垛工艺包 信号绑定
        //插入新的 码垛工艺包
        palletProcess.signal_insert_newPalletProcess.connect(signal_insert_newPalletProcess_main)
        palletProcess.signal_refreshData_PalletProcess.connect(signal_refreshData_PalletProcess_main)
        palletProcess.signal_deleteDbRow_palletProcess.connect(signal_deleteDbRow_palletProcess_main)
        palletProcess.signal_updateDbRow_palletProcess.connect(signal_updateDbRow_palletProcess_main)
        palletProcess.signal_getTool_User_PointNameToLoadCombox.connect(signal_getTool_User_PointNameToLoadCombox_main)
    }

    //启动暂停回调函数
    function start_pause_signal_fun(value)
    {
        if(value)
        {
            //切换自动
            btn_start.color= "green"
            btn_start.buttonText="运行"
        }
        else
        {
            btn_start.buttonText="运行"
            btn_start.color= "#21252d" // 默认背景
        }
    }

    // 添加报警日志的函数
    function addErrorLog(text) {
        errorLogModel.append({"errLogText": text})
        rect_alarmMesAndStatus.height= main_rect.height*0.3
        text_alarm_disp_btn.text="折叠"
    }

    //清除报警日志
    function clearErrorLog()
    {
        errorLogModel.clear()
    }

    // 定义 槽函数，用于处理初始化完成的回调
    function init_success_signal_func(value) {
        if(value==1)
        {
            text_appStatus.color="green"
            text_appStatus.text='初始化成功'
        }
        else
        {
            text_appStatus.color="red"
            text_appStatus.text='初始化失败'
        }
    }

    //手自动切换
    function manuel_auto_signal_func(value) {
        if(value)
        {
            //切换自动
            switch_autoManuel.color="#568af2"
            switch_autoManuel.buttonText="自动"
            text_appStatus.text='自动模式'
            switch_autoManuel.buttonStatus=true
        }
        else
        {
            //切为手动
            switch_autoManuel.color="#e58364"
            switch_autoManuel.buttonText="手动"
            text_appStatus.text='手动模式'
            switch_autoManuel.buttonStatus=false
        }
    }

    //主控定义信号
    signal signal_manuel_auto_switch(bool mode)
    signal signal_start_pause_switch(bool mode)
    signal signal_clear_errorList()
    //打开关闭fr_robotManuel监控
    signal signal_robotManuelSwitch(bool _switch)
    //打开关闭RobotIoSetSwitch
    signal signal_IoSetSwitch(bool _switch)

    //robotManual信号绑定 mainQML信号
    signal mySignal()
    //JOG
    signal signal_StartJOG_main(int nb,int dir)
    signal signal_StopJOG_main()
    //robot workSpce改变 绑定子qml RobotManal
    signal signal_WorkSpaceChange_main(int ref_workSpace)
    //工具坐标系改变
    signal signal_toolChange_main(string tool_name)
    //用户坐标系改变
    signal signal_userChange_main(string user_name)
    //点位执行绑定到主信号
    signal signal_pointMove_main(string pointName, string moveType,int exeFlag)
    //伺服上电断电
    signal signal_robotEnable_main(int state_robot)
    //改变robot速度值
    signal signal_robotSpeed_main(double speed_val)


    //robot点表qml信号绑定
    //删除
    signal signal_deleteDbRow_robotPointList_main(string name_str)
    //更新
    signal signal_updateDbRow_robotPointList_main(string name_str,list<string> str_list)
    //数据库新增一行
    signal signal_insert_row_robotPointList_main(string name_pt)
    //数据库刷新到 tableView信号
    signal signal_refresh_DBPointToTableView_main()
    //示教信号
    signal signal_point_teach_main(string point_name)
    //点位执行 joint运动 flag为1时执行，为0时，终止运动
    signal signal_execute_point_main(string point_name,int flag)

    //robot工具坐标系表qml信号绑定
    //删除
    signal signal_deleteDbRow_robotToolList_main(string name_str)
    //更新
    signal signal_updateDbRow_robotToolList_main(string name_str,list<string> str_list)
    //数据库新增一行
    signal signal_insert_row_robotToolList_main(string name_pt)
    //数据库刷新到 tableView信号
    signal signal_refresh_DBToolToTableView_main()
    //工具坐标系 标定新工具坐标系 示教点位
    signal signal_teach_tool_point_main(int point_index)
    //计算新工具平移变换信号绑定
    signal signal_calculate_tool_trans_main()
    //将新工具平移变换 更新到界面及数据库信号绑定
    signal signal_update_tool_trans_main(string name_tool)
    //计算工具旋转变换
    signal signal_calculate_tool_rotate_main()
    //更新工具旋转变换
    signal signal_update_tool_rotate_main(string name_tool)

    //robot用户坐标系表qml信号绑定
    //删除
    signal signal_deleteDbRow_robotUserList_main(string name_str)
    //更新
    signal signal_updateDbRow_robotUserList_main(string name_str,list<string> str_list)
    //数据库新增一行
    signal signal_insert_row_robotUserList_main(string name_pt)
    //数据库刷新到 tableView信号
    signal signal_refresh_DBUserToTableView_main()
    //示教点位
    signal signal_teach_user_point_userTable_main(int point_index)
    //计算新用户位姿变换
    signal signal_calculate_user_coord_userTable_main()
    //更新新用户在基坐标系位姿
    signal signal_update_user_coord_userTable_main(string name_tool)

    //RobotIoSet
    signal signal_SetDO_RobotIoSet_main(int id_, int status_)
    //更新输入输出标签Robot tag
    signal signal_update_RobotDIO_tag_main(string type_,string id_,string tag_)

    //palletProcess码垛工艺包
    //增
    signal signal_insert_newPalletProcess_main(string name_str)
    //刷新palletProcess数据到界面显示
    signal signal_refreshData_PalletProcess_main()
    //删除行
    signal signal_deleteDbRow_palletProcess_main(string name_str)
    //更新
    signal signal_updateDbRow_palletProcess_main(string name_str,list<string> str_list)
    //发送 获取 工具 用户名称信号 并回调
    signal signal_getTool_User_PointNameToLoadCombox_main()

}

