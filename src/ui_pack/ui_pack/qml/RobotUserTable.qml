import QtQuick
import Qt.labs.qmlmodels
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.VirtualKeyboard 2.2
import QtQuick.VirtualKeyboard.Settings 2.2

Rectangle{
    id: root_rec
    width: 800
    height: 500
    color: "#343b48"
    visible: true

    //工具坐标系
    RobotUserCalib{
        id: robotUserCalib
        anchors.fill: parent
        color: "#1b1e23"
        visible: false
        z: 99
    }

    Row{
        spacing:20
        anchors.top:parent.top
        anchors.topMargin:10
        anchors.left:parent.left
        anchors.leftMargin:50

        //文本框
        Rectangle {
            width: 200
            height: 30
            color: "lightgray"
            Layout.alignment: Qt.AlignVCenter

            TextField {
                id: pointName_text
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
            buttonText: "新增点位"
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
                    if(pointName_text.text!="")
                    {
                        //发送插入新行信号
                        signal_insert_row_robotUserList(pointName_text.text)
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

                    signal_refresh_DBUserToTableView()
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
        anchors.topMargin:50

        anchors.fill: parent
        ScrollBar.horizontal.policy: ScrollBar.AlwaysOn
        ScrollBar.vertical.policy: ScrollBar.AlwaysOn
        ScrollBar.horizontal.interactive: true
        ScrollBar.vertical.interactive: true

        ColumnLayout{
            //width: 1000
            height: height_tableRec

            //表格
            Rectangle{
                id:header_rect
                color: "#343b48"
                Layout.alignment: Qt.AlignTop
                width: 1400
                height: height_tableRec

                Column
                {
                    spacing:2
                    //表头
                    Row{
                        Layout.alignment: Qt.AlignLeft
                        spacing: 1
                        Repeater{
                            model: ["option1","option2","option3","name","comment",
                            "x","y","z","xRot","yRot","zRot"]

                            Rectangle{
                                width: {
                                    var w = 0
                                    switch(index){
                                        case 0: w = 100;break;
                                        case 1: w = 100;break;
                                        case 2: w = 100;break;
                                        case 3: w = 100;break;
                                        case 4: w = 100;break;
                                        case 5: w = 100;break;
                                        case 6: w = 100;break;
                                        case 7: w = 100;break;
                                        case 8: w = 100;break;
                                        case 9: w = 100;break;
                                        case 10: w = 100;break;

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
                            TableModelColumn { display: "x" }
                            TableModelColumn { display: "y" }
                            TableModelColumn { display: "z" }
                            TableModelColumn { display: "xRot" }
                            TableModelColumn { display: "yRot" }
                            TableModelColumn { display: "zRot" }
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
                                            selectRow(row) //选中行操作
                                            user_edit(name_selected)
                                        }
                                    }
                                }
                            }

                            //name
                            DelegateChoice{
                                column: 3
                                delegate: Rectangle {
                                    implicitWidth: 100
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
                                    implicitWidth: 100
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

                            //x
                            DelegateChoice{
                                column: 5
                                delegate: Rectangle {
                                    implicitWidth: 100
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
                            //y
                            DelegateChoice{
                                column: 6
                                delegate: Rectangle {
                                    implicitWidth: 100
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
                            //z
                            DelegateChoice{
                                column: 7
                                delegate: Rectangle {
                                    implicitWidth: 100
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
                            //xRot
                            DelegateChoice{
                                column: 8
                                delegate: Rectangle {
                                    implicitWidth: 100
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
                            //yRot
                            DelegateChoice{
                                column: 9
                                delegate: Rectangle {
                                    implicitWidth: 100
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
                            //zRot
                            DelegateChoice{
                                column: 10
                                delegate: Rectangle {
                                    implicitWidth: 100
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
        y: root_rec.height+200   // 默认让其处于窗口最下方,貌似隐藏一样
        width: root_rec.width*0.9

        visible: true       // 一直显示

        states: State
        {
            name: "visible"
            when: inputPanelID.active
            PropertyChanges
            {
                target: inputPanelID
                y: root_rec.height-inputPanelID.height-20
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

     Component.onCompleted: {
        //点位示教信号
        robotUserCalib.signal_teach_user_point.connect(signal_teach_user_point_userTable)
        //计算新用户坐标系在基坐标系变换信号
        robotUserCalib.signal_calculate_user_coord.connect(signal_calculate_user_coord_userTable)
        //将新用户坐标系在基坐标系变换 更新到界面及数据库
        robotUserCalib.signal_update_user_coord.connect(signal_update_user_coord_userTable)

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
    //工具坐标系 自定义新工具
    function user_edit(name_select)
    {
        robotUserCalib.name_selected = name_select
        robotUserCalib.visible = true
    }

    //向tableView添加数据： 数据库内容更新到tableView
    function appendRow_tableModel(value)
    {
        //添加数据
        tableModel.appendRow(
        {"update":false,"delete":false,"edit":false,"name":value[0],"comment":value[1] ,
        "x":parseFloat(value[2]),"y":parseFloat(value[3]),"z":parseFloat(value[4]),
        "xRot":parseFloat(value[5]),"yRot":parseFloat(value[6]),"zRot":parseFloat(value[7])})
        //表格rectangle框增高一行的高度
        height_tableRec+=51
        //tableView增高一行的高度
        height_tableView=height_tableView+51

    }

    //当插入新行到数据库成功后 python发送信号给qml，调用此函数给table model也添加行
    function insertNewRowToTM_DB(name_str)
    {
        console.log("tableView插入新增一行")
        tableModel.appendRow(
        {"update":false,"delete":false,"edit":false,"name":name_str,"comment":"注释",
        "x":0,"y":0,"z":0,"xRot":0,"yRot":0,"zRot":0})
        //表格rectangle框增高一行的高度
        height_tableRec+=51
        //tableView增高一行的高度
        height_tableView=height_tableView+51
        text_prompt.text="提示信息：点位插入完成"
    }

    //tableView删除一行数据
    function deleteRow_tableView()
    {
        //发送信号 数据库删除一行
        signal_deleteDbRow_robotUserList(name_selected)
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
        var list_var = [String(_row['comment']),String(_row['x']),String(_row['y']),String(_row['z']),
            String(_row['xRot']),String(_row['yRot']),String(_row['zRot'])]
         //var list_var=['0','0','0','0','0','0','0','0','0','0','0','0','0','0','0']
         console.log(list_var)
        //发送信号 给数据库
        signal_updateDbRow_robotUserList(_row['name'],list_var)
        text_prompt.text="提示信息：更新一行完成"
    }

    //新用户示教点位
    function teach_point_callBack_userTable(list_pose_data,index)
    {
        robotUserCalib.teach_point_callBack(list_pose_data,index)
    }

    //新用户计算结果回传
    function calculate_user_callback_userTable(list_data)
    {
        robotUserCalib.calculate_user_callback(list_data)
    }

    //新用户结果回传到 界面显示
    function updateUserData_calibUser_callback(list_data)
    {
        var _row = tableModel.getRow(rowIndex_selected)
        //索引出当前行数据 设置索引行数据
        tableModel.setRow(rowIndex_selected,
            {"update":false,"delete":false,"edit":false,"name":_row["name"],"comment":_row["comment"] ,
            "x":parseFloat(list_data[0]),"y":parseFloat(list_data[1]),"z":parseFloat(list_data[2]),
            "xRot":parseFloat(list_data[3]),"yRot":parseFloat(list_data[4]),"zRot":parseFloat(list_data[5])
            })
    }


//    //将数据库的数据刷新到 tableModel
//    function refresh_DBPointListToTableModel(AllRows)
//    {
//        for(row in AllRows)
//        {
//            appendRow_tableModel(row)
//        }
//        text_prompt.text="提示信息：点位数据刷新完成"
//    }

    //定义信号
    //删除
    signal signal_deleteDbRow_robotUserList(string name_str)
    //更新
    signal signal_updateDbRow_robotUserList(string name_str,list<string> str_list)
    //数据库新增一行
    signal signal_insert_row_robotUserList(string name_pt)
    //数据库刷新到 tableView信号
    signal signal_refresh_DBUserToTableView()
    //示教点位
    signal signal_teach_user_point_userTable(int point_index)
    //计算新用户位姿变换
    signal signal_calculate_user_coord_userTable()
    //更新新用户在基坐标系位姿
    signal signal_update_user_coord_userTable(string name_tool)

    //属性
    property int height_tableRec :3000
    property int height_tableView:100
    property int rowIndex_selected:-1
    property string name_selected: ""


}


