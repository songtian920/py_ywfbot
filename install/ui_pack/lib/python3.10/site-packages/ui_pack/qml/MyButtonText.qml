import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: btn_mainRect
    width: widthRect
    height: heightRect
    color: "#21252d" // 默认背景
    radius: 10
    border.color: "transparent"

    // 鼠标悬浮时的背景色变化
    MouseArea {
        id: mouseArea
        anchors.fill: parent
        hoverEnabled: true // 设置为true以启用悬停事件
        onEntered: {

        }
        onExited: {

            }
        onClicked: {
            // 按钮点击处理逻辑
        }
    }

    // 文字部分
    Text {
        id: text_btn1
        anchors.centerIn: parent
        text: buttonText // 绑定到外部传入的属性
        font.pointSize: pointSize
        color: "white" // 字体颜色为白色
        //verticalAlignment: Text.AlignVCenter

        Layout.fillWidth: true

    }

    // 定义一个属性来传递按钮文本
    property string buttonText: ""
    property int widthRect: 0
    property int heightRect: 0
    property bool buttonStatus: false
    property int pointSize:20
}
