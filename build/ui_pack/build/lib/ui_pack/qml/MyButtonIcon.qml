import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: btn_mainRect
    width: widthRect
    height: heightRect
    color: "#21252d" // 默认背景色为蓝色
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
    Image {
        id: btn_image
        anchors.centerIn:parent
        //Layout.alignment: parent.AlignHCenter
        source: imageSourcePath // 替换为实际的图片路径
        fillMode: Image.PreserveAspectFit
        height: parent.height * 0.6 // 确保图片垂直居中
    }
    RowLayout {
        anchors.fill: parent
        anchors.centerIn: parent
        // 将锚点的水平中心与父项的水平中心对齐
        //anchors.left:parent.left
        //anchors.leftMargin: 10
        spacing: 5
        // 图片部分

    }
    property int widthRect: 0
    property int heightRect: 0
    property string imageSourcePath: ""
}
