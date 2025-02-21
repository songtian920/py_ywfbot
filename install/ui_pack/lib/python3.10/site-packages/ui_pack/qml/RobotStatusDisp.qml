import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ColumnLayout{
    spacing: 20
    //机械手臂速度
    Row{
        spacing: 15
        Layout.alignment: Qt.AlignHCenter

        //减小按钮
        MyButtonText{
            id:robot_speed_decrease_btn
            widthRect: 80
            heightRect: 40
            buttonText:'手臂减速'
            pointSize:12
            color:"#1b1e23"  //初始设置为主页
            Layout.alignment: Qt.AlignVCenter
            // 鼠标悬浮时的背景色变化
            MouseArea {
                anchors.fill: parent
                hoverEnabled: true // 设置为true以启用悬停事件
                onPressed: {
                    robot_speed_decrease_btn.color = "#dce1ec"
                }
                onReleased: {
                    robot_speed_decrease_btn.color = "#1b1e23"
                }
                onClicked: {
                    //console.log("clicked")
                }
            }
        }

        //显示框
        Rectangle{
            width: 40
            height: 40
            color: "#21252d" // 默认背景
            radius: 10
            border.color: "transparent"
            Layout.alignment: Qt.AlignVCenter
            //显示值
            Text {
                text: robot_speed
                font.pointSize:10
                color: "white" // 字体颜色为白色
                anchors.centerIn: parent
            }
        }
        //增加按钮
        MyButtonText{
            id:robot_speed_increase_btn
            widthRect: 80
            heightRect: 40
            buttonText:'手臂加速'
            pointSize:12
            color:"#1b1e23"  //初始设置为主页
            Layout.alignment: Qt.AlignVCenter
            // 鼠标悬浮时的背景色变化
            MouseArea {
                anchors.fill: parent
                hoverEnabled: true // 设置为true以启用悬停事件
                onPressed: {
                    robot_speed_increase_btn.color = "#dce1ec"
                }
                onReleased: {
                    robot_speed_increase_btn.color = "#1b1e23"
                }
                onClicked: {
                    //console.log("clicked")
                }
            }
        }
    }

    //车体速度
    Row{
        spacing: 15
        Layout.alignment: Qt.AlignHCenter

        //减小按钮
        MyButtonText{
            id:amr_speed_decrease_btn
            widthRect: 80
            heightRect: 40
            buttonText:'车体减速'
            pointSize:12
            color:"#1b1e23"  //初始设置为主页
            Layout.alignment: Qt.AlignVCenter
            // 鼠标悬浮时的背景色变化
            MouseArea {
                anchors.fill: parent
                hoverEnabled: true // 设置为true以启用悬停事件
                onPressed: {
                    amr_speed_decrease_btn.color = "#dce1ec"
                }
                onReleased: {
                    amr_speed_decrease_btn.color = "#1b1e23"
                }
                onClicked: {
                    //console.log("clicked")
                }
            }
        }

        //显示框
        Rectangle{
            width: 40
            height: 40
            color: "#21252d" // 默认背景
            radius: 10
            border.color: "transparent"
            Layout.alignment: Qt.AlignVCenter
            //显示值
            Text {
                text: amr_speed
                font.pointSize:10
                color: "white" // 字体颜色为白色
                anchors.centerIn: parent
            }
        }

        //增加按钮
        MyButtonText{
        id:amr_speed_increase_btn
            widthRect: 80
            heightRect: 40
            buttonText:'车体加速'
            pointSize:12
            color:"#1b1e23"  //初始设置为主页
            Layout.alignment: Qt.AlignVCenter
            // 鼠标悬浮时的背景色变化
            MouseArea {
                anchors.fill: parent
                hoverEnabled: true // 设置为true以启用悬停事件
                onPressed: {
                    amr_speed_increase_btn.color = "#dce1ec"
                }
                onReleased: {
                    amr_speed_increase_btn.color = "#1b1e23"
                }
                onClicked: {
                    //console.log("clicked")
                }
            }
        }
    }

    ColumnLayout{
        spacing: 10

        //环境匹配分数
        Row{
            spacing: 30
            width: 150
            Layout.alignment: Qt.AlignHCenter
            //标题
            Text {
                Layout.alignment: Qt.AlignVCenter
                text: "环境匹配度：" // 绑定到外部传入的属性
                font.pointSize: 12
                color: "white" // 字体颜色为白色
            }
            //显示框
            Rectangle{
                width: 80
                height: 30
                color: "#21252d" // 默认背景
                radius: 10
                border.color: "transparent"
                Layout.alignment: Qt.AlignVCenter
                //显示值
                Text {
                    text: env_matching_score
                    font.pointSize:10
                    color: "white" // 字体颜色为白色
                    anchors.centerIn: parent
                }
            }
        }

        //手臂使能
        Row{
            spacing: 50
            Layout.alignment: Qt.AlignHCenter
            //标题
            Text {
                Layout.alignment: Qt.AlignVCenter
                text: "手臂使能：" // 绑定到外部传入的属性
                font.pointSize: 12
                color: "white" // 字体颜色为白色
            }
            //显示框
            Rectangle{
                width: 80
                height: 30
                color: "#21252d" // 默认背景
                radius: 10
                border.color: "transparent"
                Layout.alignment: Qt.AlignVCenter
                //显示值
                Text {
                    text: robot_enable
                    font.pointSize:10
                    color: "white" // 字体颜色为白色
                    anchors.centerIn: parent
                }
            }
        }

        //车体使能
        Row{
            spacing: 50
            Layout.alignment: Qt.AlignHCenter
            //标题
            Text {
                Layout.alignment: Qt.AlignVCenter
                text: "车体使能：" // 绑定到外部传入的属性
                font.pointSize: 12
                color: "white" // 字体颜色为白色
            }
            //显示框
            Rectangle{
                width: 80
                height: 30
                color: "#21252d" // 默认背景
                radius: 10
                border.color: "transparent"
                Layout.alignment: Qt.AlignVCenter
                //显示值
                Text {
                    text: amr_enable
                    font.pointSize:10
                    color: "white" // 字体颜色为白色
                    anchors.centerIn: parent
                }
            }
        }

        //车体速度
        Row{
            spacing: 50
            Layout.alignment: Qt.AlignHCenter
            //标题
            Text {
                Layout.alignment: Qt.AlignVCenter
                text: "车体速度：" // 绑定到外部传入的属性
                font.pointSize: 12
                color: "white" // 字体颜色为白色
            }
            //显示框
            Rectangle{
                width: 80
                height: 30
                color: "#21252d" // 默认背景
                radius: 10
                border.color: "transparent"
                Layout.alignment: Qt.AlignVCenter
                //显示值
                Text {
                    text: amr_speed
                    font.pointSize:10
                    color: "white" // 字体颜色为白色
                    anchors.centerIn: parent
                }
            }
        }

        //车体电量
        Row{
            spacing: 50
            Layout.alignment: Qt.AlignHCenter
            //标题
            Text {
                Layout.alignment: Qt.AlignVCenter
                text: "剩余电量：" // 绑定到外部传入的属性
                font.pointSize: 12
                color: "white" // 字体颜色为白色
            }
            //显示框
            Rectangle{
                width: 80
                height: 30
                color: "#21252d" // 默认背景
                radius: 10
                border.color: "transparent"
                Layout.alignment: Qt.AlignVCenter
                //显示值
                Text {
                    text: amr_power
                    font.pointSize:10
                    color: "white" // 字体颜色为白色
                    anchors.centerIn: parent
                }
            }
        }
        //车体电流
        Row{
            spacing: 50
            Layout.alignment: Qt.AlignHCenter
            //标题
            Text {
                Layout.alignment: Qt.AlignVCenter
                text: "当前电流：" // 绑定到外部传入的属性
                font.pointSize: 12
                color: "white" // 字体颜色为白色
            }
            //显示框
            Rectangle{
                width: 80
                height: 30
                color: "#21252d" // 默认背景
                radius: 10
                border.color: "transparent"
                Layout.alignment: Qt.AlignVCenter
                //显示值
                Text {
                    text: amr_power
                    font.pointSize:10
                    color: "white" // 字体颜色为白色
                    anchors.centerIn: parent
                }
            }
        }

        //车体电压
        Row{
            spacing: 50
            Layout.alignment: Qt.AlignHCenter
            //标题
            Text {
                Layout.alignment: Qt.AlignVCenter
                text: "当前电压：" // 绑定到外部传入的属性
                font.pointSize: 12
                color: "white" // 字体颜色为白色
            }
            //显示框
            Rectangle{
                width: 80
                height: 30
                color: "#21252d" // 默认背景
                radius: 10
                border.color: "transparent"
                Layout.alignment: Qt.AlignVCenter
                //显示值
                Text {
                    text: amr_power
                    font.pointSize:10
                    color: "white" // 字体颜色为白色
                    anchors.centerIn: parent
                }
            }
        }

        //车体位置X
        Row{
            spacing: 50
            Layout.alignment: Qt.AlignHCenter
            //标题
            Text {
                Layout.alignment: Qt.AlignVCenter
                text: "车位置X：" // 绑定到外部传入的属性
                font.pointSize: 12
                color: "white" // 字体颜色为白色
            }
            //显示框
            Rectangle{
                width: 80
                height: 30
                color: "#21252d" // 默认背景
                radius: 10
                border.color: "transparent"
                Layout.alignment: Qt.AlignVCenter
                //显示值
                Text {
                    text: amr_x_pos
                    font.pointSize:10
                    color: "white" // 字体颜色为白色
                    anchors.centerIn: parent
                }
            }
        }

        //车体位置Y
        Row{
            spacing: 50
            Layout.alignment: Qt.AlignHCenter
            //标题
            Text {
                Layout.alignment: Qt.AlignVCenter
                text: "amr_x_pos：" // 绑定到外部传入的属性
                font.pointSize: 12
                color: "white" // 字体颜色为白色
            }
            //显示框
            Rectangle{
                width: 80
                height: 30
                color: "#21252d" // 默认背景
                radius: 10
                border.color: "transparent"
                Layout.alignment: Qt.AlignVCenter
                //显示值
                Text {
                    text: amr_y_pos
                    font.pointSize:10
                    color: "white" // 字体颜色为白色
                    anchors.centerIn: parent
                }
            }
        }

        //车体位置W
        Row{
            spacing: 50
            Layout.alignment: Qt.AlignHCenter
            //标题
            Text {
                Layout.alignment: Qt.AlignVCenter
                text: "amr_w_pos：" // 绑定到外部传入的属性
                font.pointSize: 12
                color: "white" // 字体颜色为白色
            }
            //显示框
            Rectangle{
                width: 80
                height: 30
                color: "#21252d" // 默认背景
                radius: 10
                border.color: "transparent"
                Layout.alignment: Qt.AlignVCenter
                //显示值
                Text {
                    text: amr_w_pos
                    font.pointSize:10
                    color: "white" // 字体颜色为白色
                    anchors.centerIn: parent
                }
            }
        }
    }


    property int env_matching_score
    property string robot_enable: '未使能'
    property int robot_speed: 0
    property string amr_enable: '未使能'
    property int amr_speed: 0
    property int amr_power: 0
    property int amr_current: 0.0
    property double amr_voltage: 0.0
    property double amr_x_pos
    property double amr_y_pos
    property double amr_w_pos
}