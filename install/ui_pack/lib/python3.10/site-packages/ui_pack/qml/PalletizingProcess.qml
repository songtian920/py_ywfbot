import QtQuick
import Qt.labs.qmlmodels
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.VirtualKeyboard 2.2
import QtQuick.VirtualKeyboard.Settings 2.2

//码垛工艺包
Rectangle{
    id: root
    width: 800
    height: 500
    color: "#343b48"
    visible: true

    //属性
    property int height_tableRec :3000
    property int height_tableView:100
    property int rowIndex_selected:-1
    property string name_selected: ""

    //设置工具坐标系 参数属性
    property list<string> list_toolName:[]
    property string toolName_paramSet:''

    //设置用户坐标系 参数属性
    property list<string> list_userName:[]
    property string userName_paramSet:''

    //设置过渡点位 参数属性
    property list<string> list_pointName:[]
    property string pointName_paramSet:''

     //定义信号
    //删除
    signal signal_deleteDbRow_palletProcess(string name_str)
    //更新
    signal signal_updateDbRow_palletProcess(string name_str,list<string> str_list)
    //数据库新增一行
    signal signal_insert_newPalletProcess(string name_pt)
    //数据库刷新到 tableView信号
    signal signal_refreshData_PalletProcess()

    //发送信号 获取 工具坐标系和用户坐标系
    signal signal_getTool_User_PointNameToLoadCombox()

    //抽屉面板
    StackLayout
    {
        id: stackLayout
        anchors.fill: parent
        visible: true
        currentIndex: 0

        //工艺包参数表
        Rectangle{
            id: process_table
            //width: 800
            //height: 500
            color: "#343b48"
            visible: true

            Row{
                spacing:20
                anchors.top:parent.top
                anchors.topMargin:20
                anchors.left:parent.left
                anchors.leftMargin:20

                //按钮功能区
                Rectangle {
                    width: 200
                    height: 30
                    color: "lightgray"
                    Layout.alignment: Qt.AlignVCenter
                    //process名称
                    TextField {
                        id: processName_text
                        anchors.fill: parent
                        anchors.margins: 2
                        placeholderText: ""
                        color: "black"
                        //focus: true
                        selectByMouse: true
                        onAccepted: {
                            // 当用户按下回车键时，可以在这里处理输入内容
                            console.log(text)
                        }
                    }
                }
                //新增按钮
                MyButtonText{
                    id: insert_btn
                    width: 100
                    height:30
                    buttonText: "新增码垛工艺"
                    pointSize:10
                    color:"#21252d"
                    Layout.alignment: Qt.AlignVCenter
                    // 鼠标悬浮时的背景色变化
                    MouseArea {
                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            insert_btn.color = "#dce1ec"
                        }
                        onReleased: {
                            insert_btn.color = "#21252d"
                        }
                        onClicked: {
                            //新增一行
                            if(processName_text.text!="")
                            {
                                //发送插入新行信号
                                signal_insert_newPalletProcess(processName_text.text)
                                console.log("插入新增一行")
                            }
                        }
                    }
                }
                //刷新数据库 信息到 table model
                MyButtonText{
                    id: refreshDB_to_tableModel_btn
                    width: 100
                    height:30
                    buttonText: "刷新数据"
                    pointSize:10
                    color:"#21252d"
                    Layout.alignment: Qt.AlignVCenter
                    // 鼠标悬浮时的背景色变化
                    MouseArea {
                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            refreshDB_to_tableModel_btn.color = "#dce1ec"
                        }
                        onReleased: {
                            refreshDB_to_tableModel_btn.color = "#21252d"
                        }
                        onClicked: {
                            tableModel.clear()
                            signal_refreshData_PalletProcess()
                        }
                    }
                }
                //返回
                MyButtonText{
                    id: backToMenu_btn
                    width: 100
                    height:30
                    buttonText: "返回"
                    pointSize:10
                    color:"#21252d"
                    Layout.alignment: Qt.AlignVCenter
                    // 鼠标悬浮时的背景色变化
                    MouseArea {
                        anchors.fill: parent
                        hoverEnabled: true // 设置为true以启用悬停事件
                        onPressed: {
                            backToMenu_btn.color = "#dce1ec"
                        }
                        onReleased: {
                            backToMenu_btn.color = "#21252d"
                        }
                        onClicked: {
                            root.visible=false
                        }
                    }
                }

                Text {
                    id: text_prompt
                    text: '提示信息:' // 绑定到外部传入的属性
                    font.pointSize: 15
                    color: "green" // 字体颜色为红色
                    Layout.fillWidth: true
                }
            }
            ScrollView {
                anchors.top: parent.top
                anchors.topMargin:80
                anchors.fill: parent
                ScrollBar.horizontal.policy: ScrollBar.AlwaysOn
                ScrollBar.vertical.policy: ScrollBar.AlwaysOn
                ScrollBar.horizontal.interactive: true
                ScrollBar.vertical.interactive: true

                ColumnLayout{
                    //width: 1000
                    height: root.height_tableRec
                    //表格
                    Rectangle{
                        id:header_rect
                        color: "#343b48"
                        Layout.alignment: Qt.AlignTop
                        width: 2600
                        height: root.height_tableRec
                        Column
                        {
                            spacing:2
                            //表头
                            Row{
                                Layout.alignment: Qt.AlignLeft
                                spacing: 1
                                Repeater{
                                    model: ["功能1","功能2","功能3","名称","注释",
                                    "工具坐标系","用户坐标系","工具x偏移","工具y偏移","工具z偏移",
                                    "工具x旋转偏移","工具y旋转偏移","工具z旋转偏移",
                                    "用户x偏移","用户y偏移","用户z偏移",
                                    "用户x旋转偏移","用户y旋转偏移","用户z旋转偏移","过渡点"]

                                    Rectangle{
                                        width: {
                                            var w = 0
                                            switch(index){
                                                case 0: w = 100;break;
                                                case 1: w = 100;break;
                                                case 2: w = 100;break;
                                                case 3: w = 130;break;
                                                case 4: w = 130;break;
                                                case 5: w = 130;break;
                                                case 6: w = 130;break;
                                                case 7: w = 130;break;
                                                case 8: w = 130;break;
                                                case 9: w = 130;break;
                                                case 10: w = 130;break;
                                                case 11: w = 130;break;
                                                case 12: w = 130;break;
                                                case 13: w = 130;break;
                                                case 14: w = 130;break;
                                                case 15: w = 130;break;
                                                case 16: w = 130;break;
                                                case 17: w = 130;break;
                                                case 18: w = 130;break;
                                                case 19: w = 130;break;

                                            }
                                            return w
                                        }
                                        height: 30
                                        color: "#666666"
                                        border.width: 1
                                        border.color: "#848484"
                                        Text {
                                            text: modelData
                                            anchors.centerIn: parent
                                            font.pointSize: 12
                                            color: "white"
                                        }
                                    }
                                }
                            }

                            TableView {
                                id: tableView
                                clip: true
                                interactive: true
                                rowSpacing: 1
                                columnSpacing: 1
                                width:parent.width
                                height: height_tableView
                                Layout.alignment: Qt.AlignLeft
                                //anchors.top:parent.top
                                //anchors.topMargin:0
                                //anchors.left:parent.left

                                model: TableModel {
                                    id:tableModel
                                    TableModelColumn { display: "update" }
                                    TableModelColumn { display: "delete" }
                                    TableModelColumn { display: "edit" }
                                    TableModelColumn { display: "name" }
                                    TableModelColumn { display: "comment" }
                                    TableModelColumn { display: "tool" }
                                    TableModelColumn { display: "user" }
                                    TableModelColumn { display: "tool_x_offset" }
                                    TableModelColumn { display: "tool_y_offset" }
                                    TableModelColumn { display: "tool_z_offset" }
                                    TableModelColumn { display: "tool_xRot_offset" }
                                    TableModelColumn { display: "tool_yRot_offset" }
                                    TableModelColumn { display: "tool_zRot_offset" }
                                    TableModelColumn { display: "user_x_offset" }
                                    TableModelColumn { display: "user_y_offset" }
                                    TableModelColumn { display: "user_z_offset" }
                                    TableModelColumn { display: "user_xRot_offset" }
                                    TableModelColumn { display: "user_yRot_offset" }
                                    TableModelColumn { display: "user_zRot_offset" }
                                    TableModelColumn { display: "passPointName" }
                                }
                                selectionModel: ItemSelectionModel {}

                                delegate:DelegateChooser{
                                    //update
                                    DelegateChoice{
                                        column: 0
                                        delegate: Rectangle{
                                            color: "#666666"
                                            implicitWidth: 100
                                            implicitHeight: 50
                                            border.width: 1
                                            border.color: "#848484"

                                            Button{
                                                width: 70
                                                height: 25
                                                anchors.centerIn: parent
                                                text: "更新"
                                                background: Rectangle{
                                                    radius: 4
                                                    color: "cyan"
                                                }
                                                onPressed: {
                                                    color = "#dce1ec"
                                                    selectRow(row)  //选中行
                                                    text_prompt.text="提示信息："
                                                }
                                                onReleased: {
                                                    color = "#666666"
                                                }
                                                onClicked: {
                                                    console.log("btn clicked row:",row)
                                                    updateRow_tableView(row)
                                                }
                                            }
                                        }
                                    }

                                    //delete
                                    DelegateChoice{
                                        column: 1
                                        delegate: Rectangle{
                                            color: "#666666"
                                            implicitWidth: 100
                                            implicitHeight: 50
                                            border.width: 1
                                            border.color: "#848484"

                                            // 创建一个ProgressBar控件，用于显示进度
                                            ProgressBar {
                                                // 为ProgressBar分配一个唯一ID，以便在代码中引用
                                                id: progressBar
                                                // 将ProgressBar居中显示
                                                anchors.left: parent.left
                                                anchors.bottom:parent.bottom
                                                // 设置ProgressBar的宽度为父元素宽度的80%
                                                width: parent.width
                                                // 设置ProgressBar的高度
                                                height: parent.height*0.1
                                                // 初始化ProgressBar的值为0
                                                value: 0
                                                // 设置ProgressBar的最小值
                                                from: 0
                                                // 设置ProgressBar的最大值
                                                to: 100
                                            }

                                            // 创建一个Timer控件，用于逐步更新进度条的值
                                            Timer {
                                                // 为Timer分配一个唯一ID，以便在代码中引用
                                                id: timer
                                                // 设置定时器的触发间隔（毫秒）
                                                interval: 20
                                                // 初始时定时器不运行
                                                running: false
                                                // 定时器可以重复触发
                                                repeat: true
                                                // 当定时器触发时执行的操作
                                                onTriggered: {
                                                    // 计算新的进度值
                                                    var value = progressBar.value + 2
                                                    // 如果进度值超过最大值，停止定时器
                                                    if (value > 100) {
                                                        timer.stop()
                                                        deleteRow_tableView()  //删除当前选中行函数
                                                    } else {
                                                        console.log("3, value=" + value)
                                                        // 更新进度条的值
                                                        progressBar.value = value
                                                        // 更新显示进度值的文本
                                                        //progressText.text = "Progress: " + progressBar.value
                                                    }
                                                }
                                            }

                                            Button{
                                                id:delete_btn
                                                width: 70
                                                height: 25
                                                anchors.centerIn: parent
                                                text: "删除"
                                                background: Rectangle{
                                                    radius: 4
                                                    color: "cyan"
                                                }
                                                onPressed: {
                                                    //color = "#dce1ec"
                                                    selectRow(row)
                                                    progressBar.value=0
                                                    timer.start()  // 按下时启动计时器
                                                    text_prompt.text="提示信息："
                                                }
                                                onReleased: {
                                                    color = "#666666"
                                                    progressBar.value=0
                                                    timer.stop()  // 释放时停止计时器
                                                }
                                                onCanceled: {
                                                    progressBar.value=0
                                                    timer.stop()  // 取消时停止计时器
                                                }
                                            }
                                        }
                                    }

                                    //edit
                                    DelegateChoice{
                                        column: 2
                                        delegate: Rectangle{
                                            color: "#666666"
                                            implicitWidth: 100
                                            implicitHeight: 50
                                            border.width: 1
                                            border.color: "#848484"
                                            Button{
                                                width: 70
                                                height: 25
                                                anchors.centerIn: parent
                                                text: "编辑"
                                                background: Rectangle{
                                                    radius: 4
                                                    color: "cyan"
                                                }
                                                onPressed: {
                                                    color = "#dce1ec"
                                                    selectRow(row)
                                                    text_prompt.text="提示信息："
                                                }
                                                onReleased: {
                                                    color = "#666666"
                                                }
                                                onClicked: {
                                                    console.log("btn clicked row:",row)
                                                    //选中行操作
                                                    selectRow(row)

                                                    //获取工具和用户坐标加载到多选框

                                                    palletProcess_edit(name_selected)
                                                }
                                            }
                                        }
                                    }

                                    //name
                                    DelegateChoice{
                                        column: 3
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()

                                                TableView.onCommit: {
                                                    display = text
                                                    // 'display = text' is short-hand for:
                                                    // let index = TableView.view.index(row, column)
                                                    // TableView.view.model.setData(index, "display", text)
                                                }
                                            }
                                        }
                                    }

                                    //comment
                                    DelegateChoice{
                                        column: 4
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()

                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }

                                    //tool
                                    DelegateChoice{
                                        column: 5
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()

                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }

                                    //user
                                    DelegateChoice{
                                        column: 6
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()

                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }

                                    //tool_x_offset
                                    DelegateChoice{
                                        column: 7
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()
                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }
                                    //tool_y_offset
                                    DelegateChoice{
                                        column: 8
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()
                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }
                                    //tool_z_offset
                                    DelegateChoice{
                                        column: 9
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()
                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }
                                    //tool_xRot_offset
                                    DelegateChoice{
                                        column: 10
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()
                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }
                                    //tool_yRot_offset
                                    DelegateChoice{
                                        column: 11
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()
                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }
                                    //tool_zRot_offset
                                    DelegateChoice{
                                        column: 12
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()
                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }

                                    //user_x_offset
                                    DelegateChoice{
                                        column: 13
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()
                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }
                                    //user_y_offset
                                    DelegateChoice{
                                        column: 14
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()
                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }
                                    //user_z_offset
                                    DelegateChoice{
                                        column: 15
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()
                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }
                                    //user_xRot_offset
                                    DelegateChoice{
                                        column: 16
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()
                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }
                                    //user_yRot_offset
                                    DelegateChoice{
                                        column: 17
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()
                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }
                                    //user_zRot_offset
                                    DelegateChoice{
                                        column: 18
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()
                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }
                                    //passPoint
                                    DelegateChoice{
                                        column: 19
                                        delegate: Rectangle {
                                            implicitWidth: 130
                                            implicitHeight: 50
                                            required property bool selected
                                            required property bool current
                                            border.width: current ? 2 : 0
                                            color: selected ? "lightblue" : palette.base
                                            Text{
                                                text: model.display
                                                padding: 12
                                            }
                                            TableView.editDelegate: TextField {
                                                anchors.fill: parent
                                                text: display
                                                horizontalAlignment: TextInput.AlignHCenter
                                                verticalAlignment: TextInput.AlignVCenter
                                                Component.onCompleted: selectAll()
                                                TableView.onCommit: {
                                                    display = text
                                                }
                                            }
                                        }
                                    }
                                //委托结束
                                }
                            }

                        }
                    }
                }
            }


            SelectionRectangle {
                target: tableView
            }

            //键盘输入
            InputPanel
            {
                id: inputPanelID
                z: 99
                x: 10
                y: root.height+200   // 默认让其处于窗口最下方,貌似隐藏一样
                width: root.width*0.9

                visible: true       // 一直显示

                states: State
                {
                    name: "visible"
                    when: inputPanelID.active
                    PropertyChanges
                    {
                        target: inputPanelID
                        y: root.height-inputPanelID.height-20
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

        //码垛工艺参数 工具参数
        Rectangle{
            id: paramToolUserSet_Pallet
            color: "#343b48"
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
                            if(stackLayout.currentIndex >0)
                            {
                                stackLayout.currentIndex = stackLayout.currentIndex - 1
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
                            if(stackLayout.currentIndex <8)
                            {
                                stackLayout.currentIndex = stackLayout.currentIndex+1
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
                            root_toolCalib.visible = false
                        }
                    }
                }
            }
            //工具坐标系参数
            Row{
                spacing:20
                anchors.centerIn: parent
                Column{
                    spacing: 18
                    //指定工具坐标系
                    Row{
                        spacing:16
                        // 文字部分
                        Text {
                            text: "选择工具坐标系"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        //tool combox
                        //工具坐标系选择
                        ComboBox {
                            id: tool_comboBox
                            width: 100
                            height:30
                            model: toolMolde // 下拉框的选项
                            onCurrentIndexChanged: {
                                if (currentIndex !== -1)
                                {
                                    //改变工具坐标
                                    toolName_paramSet = tool_comboBox.model.get(currentIndex).name
                                }
                            }
                        }
                        ListModel {
                            id: toolMolde
                            //ListElement {name: "aaaa"; type: "AAAA"}
                            //ListElement {name: "bbbb"; type: "BBBB"}
                        }
                    }
                    //工具坐标系偏移值 tool_x_offset
                    Row{
                        spacing:72
                        Text {
                            text: "x补偿值"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        Rectangle {
                            width: 100
                            height: 30
                            color: "lightgray"

                            //x补偿值
                            TextField {
                                id: tool_x_offset_text
                                anchors.fill: parent
                                anchors.margins: 2
                                placeholderText: '0.0'
                                color: "black"
                                //focus: true
                                selectByMouse: true
                                onAccepted: {
                                    // 当用户按下回车键时，可以在这里处理输入内容
                                    console.log(text)
                                }
                            }
                        }
                    }
                    //工具坐标系偏移值 tool_y_offset
                    Row{
                        spacing:72
                        Text {

                            text: "y补偿值"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        Rectangle {
                            width: 100
                            height: 30
                            color: "lightgray"

                            //y补偿值
                            TextField {
                                id: tool_y_offset_text
                                anchors.fill: parent
                                anchors.margins: 2
                                placeholderText: '0.0'
                                color: "black"
                                //focus: true
                                selectByMouse: true
                                onAccepted: {
                                    // 当用户按下回车键时，可以在这里处理输入内容
                                    console.log(text)
                                }
                            }
                        }
                    }
                    //工具坐标系偏移值 tool_z_offset
                    Row{
                        spacing:72
                        Text {

                            text: "z补偿值"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        Rectangle {
                            width: 100
                            height: 30
                            color: "lightgray"

                            //z补偿值
                            TextField {
                                id: tool_z_offset_text
                                anchors.fill: parent
                                anchors.margins: 2
                                placeholderText: '0.0'
                                color: "black"
                                //focus: true
                                selectByMouse: true
                                onAccepted: {
                                    // 当用户按下回车键时，可以在这里处理输入内容
                                    console.log(text)
                                }
                            }
                        }
                    }
                    //工具坐标系偏移值 tool_xRot_offset
                    Row{
                        spacing:40
                        Text {

                            text: "x旋转补偿值"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        Rectangle {
                            width: 100
                            height: 30
                            color: "lightgray"

                            //x旋转补偿值
                            TextField {
                                id: tool_xRot_offset_text
                                anchors.fill: parent
                                anchors.margins: 2
                                placeholderText: '0.0'
                                color: "black"
                                //focus: true
                                selectByMouse: true
                                onAccepted: {
                                    // 当用户按下回车键时，可以在这里处理输入内容
                                    console.log(text)
                                }
                            }
                        }
                    }
                    //工具坐标系偏移值 tool_yRot_offset
                    Row{
                        spacing:40
                        Text {

                            text: "y旋转补偿值"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        Rectangle {
                            width: 100
                            height: 30
                            color: "lightgray"

                            //y旋转补偿值
                            TextField {
                                id: tool_yRot_offset_text
                                anchors.fill: parent
                                anchors.margins: 2
                                placeholderText: '0.0'
                                color: "black"
                                //focus: true
                                selectByMouse: true
                                onAccepted: {
                                    // 当用户按下回车键时，可以在这里处理输入内容
                                    console.log(text)
                                }
                            }
                        }
                    }
                    //工具坐标系偏移值 tool_zRot_offset
                    Row{
                        spacing:40
                        Text {

                            text: "z旋转补偿值"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        Rectangle {
                            width: 100
                            height: 30
                            color: "lightgray"

                            //z旋转补偿值
                            TextField {
                                id: tool_zRot_offset_text
                                anchors.fill: parent
                                anchors.margins: 2
                                placeholderText: '0.0'
                                color: "black"
                                //focus: true
                                selectByMouse: true
                                onAccepted: {
                                    // 当用户按下回车键时，可以在这里处理输入内容
                                    console.log(text)
                                }
                            }
                        }
                    }
                    Row{
                        spacing:35
                        Text {
                            text: "点击设置保存"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        //设置工具参数按钮
                        MyButtonText{
                            id:setTool_btn
                            widthRect: 100
                            heightRect: 30

                            buttonText:'设置'
                            pointSize:12
                            color: "gray"
                            MouseArea {
                                anchors.fill: parent
                                hoverEnabled: false // 设置为true以启用悬停事件
                                onPressed: {
                                    setTool_btn.color = "green"
                                }
                                onReleased: {
                                    setTool_btn.color = "gray"
                                }
                                onClicked: {

                                }
                            }
                        }
                    }
                }
                //用户坐标系设置
                Column{
                    spacing: 18
                    //指定用户坐标系
                    Row{
                        spacing:16
                        // 文字部分
                        Text {
                            text: "选择用户坐标系"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        //user combox
                        //用户坐标系选择
                        ComboBox {
                            id: user_comboBox
                            width: 100
                            height:30
                            model: userMolde // 下拉框的选项
                            onCurrentIndexChanged: {
                                if (currentIndex !== -1)
                                {
                                    //获取选框选择项的名称
                                    userName_paramSet = user_comboBox.model.get(currentIndex).name
                                }
                            }
                        }
                        ListModel {
                            id: userMolde
                            //ListElement {name: "aaaa"; type: "AAAA"}
                            //ListElement {name: "bbbb"; type: "BBBB"}
                        }
                    }
                    //工具坐标系偏移值 user_x_offset
                    Row{
                        spacing:72
                        Text {

                            text: "x补偿值"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        Rectangle {
                            width: 100
                            height: 30
                            color: "lightgray"

                            //x补偿值
                            TextField {
                                id: user_x_offset_text
                                anchors.fill: parent
                                anchors.margins: 2
                                placeholderText: '0.0'
                                color: "black"
                                //focus: true
                                selectByMouse: true
                                onAccepted: {
                                    // 当用户按下回车键时，可以在这里处理输入内容
                                    console.log(text)
                                }
                            }
                        }
                    }
                    //用户坐标系偏移值 user_y_offset
                    Row{
                        spacing:72
                        Text {
                            text: "y补偿值"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        Rectangle {
                            width: 100
                            height: 30
                            color: "lightgray"

                            //y补偿值
                            TextField {
                                id: user_y_offset_text
                                anchors.fill: parent
                                anchors.margins: 2
                                placeholderText: '0.0'
                                color: "black"
                                //focus: true
                                selectByMouse: true
                                onAccepted: {
                                    // 当用户按下回车键时，可以在这里处理输入内容
                                    console.log(text)
                                }
                            }
                        }
                    }
                    //用户坐标系偏移值 tool_z_offset
                    Row{
                        spacing:72
                        Text {

                            text: "z补偿值"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        Rectangle {
                            width: 100
                            height: 30
                            color: "lightgray"

                            //z补偿值
                            TextField {
                                id: user_z_offset_text
                                anchors.fill: parent
                                anchors.margins: 2
                                placeholderText: '0.0'
                                color: "black"
                                //focus: true
                                selectByMouse: true
                                onAccepted: {
                                    // 当用户按下回车键时，可以在这里处理输入内容
                                    console.log(text)
                                }
                            }
                        }
                    }
                    //用户坐标系偏移值 tool_xRot_offset
                    Row{
                        spacing:40
                        Text {

                            text: "x旋转补偿值"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        Rectangle {
                            width: 100
                            height: 30
                            color: "lightgray"

                            //x旋转补偿值
                            TextField {
                                id: user_xRot_offset_text
                                anchors.fill: parent
                                anchors.margins: 2
                                placeholderText: '0.0'
                                color: "black"
                                //focus: true
                                selectByMouse: true
                                onAccepted: {
                                    // 当用户按下回车键时，可以在这里处理输入内容
                                    console.log(text)
                                }
                            }
                        }
                    }
                    //工具坐标系偏移值 tool_yRot_offset
                    Row{
                        spacing:40
                        Text {

                            text: "y旋转补偿值"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        Rectangle {
                            width: 100
                            height: 30
                            color: "lightgray"

                            //y旋转补偿值
                            TextField {
                                id: user_yRot_offset_text
                                anchors.fill: parent
                                anchors.margins: 2
                                placeholderText: '0.0'
                                color: "black"
                                //focus: true
                                selectByMouse: true
                                onAccepted: {
                                    // 当用户按下回车键时，可以在这里处理输入内容
                                    console.log(text)
                                }
                            }
                        }
                    }
                    //工具坐标系偏移值 tool_zRot_offset
                    Row{
                        spacing:40
                        Text {

                            text: "z旋转补偿值"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        Rectangle {
                            width: 100
                            height: 30
                            color: "lightgray"

                            //z旋转补偿值
                            TextField {
                                id: user_zRot_offset_text
                                anchors.fill: parent
                                anchors.margins: 2
                                placeholderText: '0.0'
                                color: "black"
                                //focus: true
                                selectByMouse: true
                                onAccepted: {
                                    // 当用户按下回车键时，可以在这里处理输入内容
                                    console.log(text)
                                }
                            }
                        }
                    }
                    Row{
                        spacing:35
                        Text {
                            text: "点击设置保存"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        //设置工具参数按钮
                        MyButtonText{
                            id:setUser_btn
                            widthRect: 100
                            heightRect: 30

                            buttonText:'设置'
                            pointSize:12
                            color: "gray"
                            MouseArea {
                                anchors.fill: parent
                                hoverEnabled: false // 设置为true以启用悬停事件
                                onPressed: {
                                    setUser_btn.color = "green"
                                }
                                onReleased: {
                                    setUser_btn.color = "gray"
                                }
                                onClicked: {

                                }
                            }
                        }
                    }
                }
                //过渡点
                Column{
                    spacing: 18
                    //指定工具坐标系
                    Row{
                        spacing:16
                        // 文字部分
                        Text {
                            text: "选择过渡点位名称"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        //tool combox
                        //工具坐标系选择
                        ComboBox {
                            id: point_comboBox
                            width: 100
                            height:30
                            model: pointMolde // 下拉框的选项
                            onCurrentIndexChanged: {
                                if (currentIndex !== -1)
                                {
                                    //改变工具坐标
                                    toolName_paramSet = tool_comboBox.model.get(currentIndex).name
                                }
                            }
                        }

                        ListModel {
                            id: pointMolde
                            //ListElement {name: "aaaa"; type: "AAAA"}
                            //ListElement {name: "bbbb"; type: "BBBB"}
                        }
                    }
                    Row{
                        spacing:50
                        Text {
                            text: "点击设置保存"
                            font.pointSize: 12
                            color: "white" // 字体颜色为白色
                        }
                        //设置过渡点按钮
                        MyButtonText{
                            id:setPassPoint_btn
                            widthRect: 100
                            heightRect: 30

                            buttonText:'设置'
                            pointSize:12
                            color: "gray"
                            MouseArea {
                                anchors.fill: parent
                                hoverEnabled: false // 设置为true以启用悬停事件
                                onPressed: {
                                    setPassPoint_btn.color = "green"
                                }
                                onReleased: {
                                    setPassPoint_btn.color = "gray"
                                }
                                onClicked: {

                                }
                            }
                        }
                    }
                }
            }
        }
        //码垛工艺参数 用户坐标系参数
        Rectangle{
            id: layerData_Pallet

        }
    }

    Component.onCompleted: {

    }

    //选中行
    function selectRow(row_index)
    {
        var index=parseInt(row_index)
        console.log(parseInt(row_index))
        rowIndex_selected = index

        var row_selected = tableModel.getRow(rowIndex_selected)
        name_selected = row_selected['name']
        console.log(name_selected)
    }
    //编辑
    function palletProcess_edit(name_select)
    {
        stackLayout.currentIndex = 1
        //发送信号 通过回调 加载工具及用户参数
        signal_getTool_User_PointNameToLoadCombox()
    }

    //向tableView添加数据： 数据库内容更新到tableView
    function appendParamPalletProcess(value)
    {
        //添加数据
        tableModel.appendRow(
        {"update":false,"delete":false,"edit":false,"name":value[0],"comment":value[1] ,
        "tool":value[2],"user":value[3],
        "tool_x_offset":parseFloat(value[4]),"tool_y_offset":parseFloat(value[5]),"tool_z_offset":parseFloat(value[6]),
        "tool_xRot_offset":parseFloat(value[7]),"tool_yRot_offset":parseFloat(value[8]),"tool_zRot_offset":parseFloat(value[9]),
        "user_x_offset":parseFloat(value[10]),"user_y_offset":parseFloat(value[11]),"user_z_offset":parseFloat(value[12]),
        "user_xRot_offset":parseFloat(value[13]),"user_yRot_offset":parseFloat(value[14]),"user_zRot_offset":parseFloat(value[15]),
        "passPointName":value[16]})
        //表格rectangle框增高一行的高度
        root.height_tableRec+=51
        //tableView增高一行的高度
        height_tableView=height_tableView+51
    }

    //当插入新行到数据库成功后 python发送信号给qml，调用此函数给table model也添加行
    //码垛工艺包 插入新码垛工艺
    function insertNewParamPalletProcessToTM(name_str)
    {
        console.log("tableView插入新增一行")
        tableModel.appendRow(
        {"update":false,"delete":false,"edit":false,"name":name_str,"comment":"注释",
        "tool":"", "user":"",
        "tool_x_offset":0,"tool_y_offset":0,"tool_z_offset":0,
        "tool_xRot_offset":0,"tool_yRot_offset":0,"tool_zRot_offset":0,
        "user_x_offset":0,"user_y_offset":0,"user_z_offset":0,
        "user_xRot_offset":0,"user_yRot_offset":0,"user_zRot_offset":0,
        "passPointName":""})
        //表格rectangle框增高一行的高度
        root.height_tableRec+=51
        //tableView增高一行的高度
        height_tableView = height_tableView+51
        text_prompt.text="提示信息：点位插入完成"
    }

    //tableView删除一行数据
    function deleteRow_tableView()
    {
        //发送信号 数据库删除一行
        signal_deleteDbRow_palletProcess(name_selected)
        console.log(rowIndex_selected)
        //tableView删除一行
        tableModel.removeRow(rowIndex_selected,1)
        text_prompt.text="提示信息：删除一行完成"
    }

    //更新一行
    function updateRow_tableView(row_index)
    {
        //索引出当前行数据 转换成字符串列表， 发送信号更新数据库
        var _row = tableModel.getRow(row_index)
        console.log(String(_row['comment']))
        var list_var = [String(_row['comment']),String(_row['tool']),String(_row['user']),
            String(_row['tool_x_offset']),String(_row['tool_y_offset']),String(_row['tool_z_offset']),
            String(_row['tool_xRot_offset']),String(_row['tool_yRot_offset']),String(_row['tool_zRot_offset']),
            String(_row['user_x_offset']),String(_row['user_y_offset']),String(_row['user_z_offset']),
            String(_row['user_xRot_offset']),String(_row['user_yRot_offset']),String(_row['user_zRot_offset']),
            String(_row['passPointName'])]
        console.log(list_var)
        //发送信号 给数据库
        signal_updateDbRow_palletProcess(_row['name'],list_var)
        text_prompt.text="提示信息：更新一行完成"
    }

    //回调 获取工具,用户,点位 名称 第一步
    function getToolUserPointNames_callback(toolNames,userNames,pointNames)
    {
        list_toolName = toolNames
        list_toolName.unshift("tool0")  //增加默认工具坐标系
        list_userName = userNames
        list_userName.unshift("fr_baseLink")  //增加默认用户坐标系
        list_pointName = pointNames
        //从数据行获取 当前数据
        getParam_toolAndUser()
        //工具名称 用户名称加载到combox
        combox_loadToolUserNames()
    }

    //将编辑的工具参数及用户参数赋值给界面 属性值  第二步
    function getParam_toolAndUser()
    {
        var row_selected = tableModel.getRow(rowIndex_selected)
        toolName_paramSet = row_selected['tool']
        tool_x_offset_text.text = String(row_selected['tool_x_offset'])
        tool_y_offset_text.text = String(row_selected['tool_y_offset'])
        tool_z_offset_text.text = String(row_selected['tool_z_offset'])
        tool_xRot_offset_text.text = String(row_selected['tool_xRot_offset'])
        tool_yRot_offset_text.text = String(row_selected['tool_yRot_offset'])
        tool_zRot_offset_text.text = String(row_selected['tool_zRot_offset'])

        userName_paramSet = row_selected['user']
        user_x_offset_text.text = String(row_selected['user_x_offset'])
        user_y_offset_text.text = String(row_selected['user_y_offset'])
        user_z_offset_text.text = String(row_selected['user_z_offset'])
        user_xRot_offset_text.text = String(row_selected['user_xRot_offset'])
        user_yRot_offset_text.text = String(row_selected['user_yRot_offset'])
        user_zRot_offset_text.text = String(row_selected['user_zRot_offset'])

        pointName_paramSet = row_selected['passPointName']
    }

    //获取工具坐标系，并加载到toolModel  第三步
    function combox_loadToolUserNames()
    {
        //工具多选框加载
        toolMolde.clear()
        for(var val of list_toolName)
        {
            toolMolde.append({name: val})
        }
        //工具多择框初始值设置
        for (let i = 0; i < toolMolde.count; i++) {
            if (toolMolde.get(i).name === toolName_paramSet) {
                tool_comboBox.currentIndex = i;
                break;
            }
        }

        //用户多选框加载
        userMolde.clear()
        for(var val of list_userName)
        {
            userMolde.append({name: val})
        }
        //用户多择框初始值设置
        for (let i = 0; i < userMolde.count; i++) {
            if (userMolde.get(i).name === userName_paramSet) {
                user_comboBox.currentIndex = i;
                break;
            }
        }

        //点位多选框加载
        pointMolde.clear()
        for(var val of list_pointName)
        {
            pointMolde.append({name: val})
        }
        //用户多择框初始值设置
        for (let i = 0; i < pointMolde.count; i++) {
            if (pointMolde.get(i).name === pointName_paramSet) {
                point_comboBox.currentIndex = i;
                break;
            }
        }
    }

    //工具坐标系参数保存
    function toolParamSave()
    {
        var row_selected = tableModel.getRow(rowIndex_selected)
        //索引出当前行数据 设置索引行数据
        tableModel.setRow(rowIndex_selected,
        {"update":false,"delete":false,"edit":false,"name":row_selected['name'],"comment":row_selected['comment'],
        "tool":tool_comboBox.currentText, "user":row_selected['user'],
        "tool_x_offset":parseFloat(tool_x_offset_text.text) ,"tool_y_offset":parseFloat(tool_y_offset_text.text),"tool_z_offset":parseFloat(tool_z_offset_text.text),
        "tool_xRot_offset":parseFloat(tool_xRot_offset_text.text),"tool_yRot_offset":parseFloat(tool_yRot_offset_text.text),"tool_zRot_offset":parseFloat(tool_zRot_offset_text.text),
        "user_x_offset":row_selected['user_x_offset'],"user_y_offset":row_selected['user_y_offset'],"user_z_offset":row_selected['user_z_offset'],
        "user_xRot_offset":row_selected['user_xRot_offset'],"user_yRot_offset":row_selected['user_yRot_offset'],"user_zRot_offset":row_selected['user_zRot_offset'],
        "passPointName":row_selected['passPointName']})

        //更新当前行
        updateRow_tableView(rowIndex_selected)
    }
    //用户坐标系参数保存
    function userParamSave()
    {
        var row_selected = tableModel.getRow(rowIndex_selected)
        //索引出当前行数据 设置索引行数据
        tableModel.setRow(rowIndex_selected,
        {"update":false,"delete":false,"edit":false,"name":row_selected['name'],"comment":row_selected['comment'],
        "tool":row_selected['tool'], "user":user_comboBox.currentText,
        "tool_x_offset":row_selected['tool_x_offset'] ,"tool_y_offset":row_selected['tool_y_offset'],"tool_z_offset":row_selected['tool_z_offset'],
        "tool_xRot_offset":row_selected['tool_xRot_offset'],"tool_yRot_offset":row_selected['tool_yRot_offset'],"tool_zRot_offset":row_selected['tool_zRot_offset'],
        "user_x_offset":parseFloat(user_x_offset_text.text),"user_y_offset":parseFloat(user_y_offset_text.text),"user_z_offset":parseFloat(user_z_offset_text.text),
        "user_xRot_offset":parseFloat(user_xRot_offset_text.text),"user_yRot_offset":parseFloat(user_yRot_offset_text.text),"user_zRot_offset":parseFloat(user_zRot_offset_text.text),
        "passPointName":row_selected['passPointName']})

        //更新当前行
        updateRow_tableView(rowIndex_selected)
    }
    //栈板过渡点保存
    function passPointNameSave()
    {
        var row_selected = tableModel.getRow(rowIndex_selected)
        //索引出当前行数据 设置索引行数据
        tableModel.setRow(rowIndex_selected,
        {"update":false,"delete":false,"edit":false,"name":row_selected['name'],"comment":row_selected['comment'],
        "tool":row_selected['tool'], "user":row_selected['user'],
        "tool_x_offset":row_selected['tool_x_offset'] ,"tool_y_offset":row_selected['tool_y_offset'],"tool_z_offset":row_selected['tool_z_offset'],
        "tool_xRot_offset":row_selected['tool_xRot_offset'],"tool_yRot_offset":row_selected['tool_yRot_offset'],"tool_zRot_offset":row_selected['tool_zRot_offset'],
        "user_x_offset":row_selected['user_x_offset'],"user_y_offset":row_selected['user_y_offset'],"user_z_offset":row_selected['user_z_offset'],
        "user_xRot_offset":row_selected['user_xRot_offset'],"user_yRot_offset":row_selected['user_yRot_offset'],"user_zRot_offset":row_selected['user_zRot_offset'],
        "passPointName":point_comboBox.currentText})

        //更新当前行
        updateRow_tableView(rowIndex_selected)
    }
}