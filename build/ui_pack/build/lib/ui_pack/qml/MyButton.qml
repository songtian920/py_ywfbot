import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: btn_mainRect
    width: widthRect
    height: heightRect
    color: "#1b1e23" // 默认背景色为蓝色
    radius: 10
    border.color: "transparent"

    // 鼠标悬浮时的背景色变化
    MouseArea {
        id: mouseArea
        anchors.fill: parent
        onClicked: {
            // 按钮点击处理逻辑
        }
    }

    // 图片部分
    Image {
        id: btn_image
        source: imageSourcePath // 替换为实际的图片路径
        fillMode: Image.PreserveAspectFit
        height: parent.height * 0.6 // 确保图片垂直居中
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: parent.left
        anchors.leftMargin: 10
        //width: parent.width/2
    }

    // 文字部分
    Text {
        text: buttonText // 绑定到外部传入的属性
        font.pointSize: 10
        color: "white" // 字体颜色为白色
        verticalAlignment: Text.AlignVCenter
        Layout.fillWidth: true
        anchors.verticalCenter: parent.verticalCenter
        //anchors.centerIn:parent
        anchors.left: parent.left
        anchors.leftMargin: parent.width/3
        visible: visible_textBtn
    }



    // 定义一个属性来传递按钮文本
    property string buttonText: ""
    property int widthRect: 0
    property int heightRect: 0
    property string imageSourcePath: ""
    property bool visible_textBtn: true
}
