import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.5

import QtQuick.VirtualKeyboard 2.2
import QtQuick.VirtualKeyboard.Settings 2.2

Rectangle{
    id: root
    visible: true
    width: 800
    height: 600
    //title: qsTr("Hello World")

    ColumnLayout
    {
        anchors.top: parent.top
        anchors.topMargin: root.height * 0.2
        anchors.horizontalCenter: parent.horizontalCenter
        spacing: 25

        RowLayout
        {
            spacing: 25

            Text
            {
                text: qsTr("用户名:")
                font.family: "微软雅黑"
                font.pixelSize: 20
            }
            TextField
            {
                placeholderText: "输入用户名.."
                font.family: "微软雅黑"
                font.pixelSize: 16
                Layout.preferredWidth: root.width * 0.25

                background: Rectangle
                {
                    radius: 4
                    border.color: parent.focus  ? "#498ff8" : "#C4DBFC"
                }
            }
        }

        RowLayout
        {
            spacing: 25

            Text
            {
                text: qsTr("密   码:")
                font.family: "微软雅黑"
                font.pixelSize: 20
            }
            TextField
            {
                placeholderText: "输入密码.."
                font.family: "微软雅黑"
                font.pixelSize: 16
                Layout.preferredWidth: root.width * 0.25

                background: Rectangle
                {
                    radius: 4
                    border.color: parent.focus  ? "#498ff8" : "#C4DBFC"
                }
            }
        }
    }

    InputPanel
    {
        id: inputPannelID
        z: 99
        y: 200     // 默认让其处于窗口最下方,貌似隐藏一样
        width: root.width
        visible: true       // 一直显示

        states: State
        {
            name: "visible"
            when: inputPannelID.active
            PropertyChanges
            {
                target: inputPannelID
                y: root.height-inputPannelID.height
            }
        }
        transitions: Transition
        {
            from: ""
            to: "visible"
            reversible: true
            ParallelAnimation
            {
                NumberAnimation
                {
                    properties: "y"
                    duration: 250
                    easing.type: Easing.InOutQuad
                }
            }
        }


        Component.onCompleted:
        {
            //VirtualKeyboardSettings.styleName = "retro"                         // 复古样式
            VirtualKeyboardSettings.wordCandidateList.alwaysVisible = true
            VirtualKeyboardSettings.activeLocales = ["en_US","zh_CN","ja_JP"]   // 英语、中文、日语 (若不设置,则语言就有很多种)
        }
    }
}
