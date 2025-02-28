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
    //color: "green"
    visible: true

    //按钮功能区
    Row{
        spacing:20
        anchors.top:parent.top
        anchors.topMargin:10
        anchors.left:parent.left
        anchors.leftMargin:50
        z:1
        //监控启停按钮
        MyButtonText{
            id: dIO_refresh_btn
            width: 100
            height:30
            buttonText: "停止监控"
            pointSize:10
            color:"green"
            Layout.alignment: Qt.AlignVCenter
            // 鼠标悬浮时的背景色变化
            MouseArea {
                anchors.fill: parent
                hoverEnabled: true // 设置为true以启用悬停事件
                onPressed: {

                }
                onReleased: {

                }
                onClicked: {
                    if(dIO_refresh_flag)
                    {
                        dIO_refresh_btn.color = "gray"
                        dIO_refresh_btn.buttonText = "开始监控"
                        dIO_refresh_flag = false
                    }
                    else
                    {
                        dIO_refresh_btn.color = "green"
                        dIO_refresh_btn.buttonText = "停止监控"
                        dIO_refresh_flag = true
                    }
                }
            }
        }

        Text {
            id: text_prompt

            text: '监控停止后可更改输入输出标签' // 绑定到外部传入的属性
            font.pointSize: 15
            color: "red" // 字体颜色为红色

            Layout.fillWidth: true
        }
    }


    ScrollView {
        anchors.top: parent.top
        anchors.topMargin:50
        anchors.left: parent.left
        anchors.leftMargin:20

        anchors.fill: parent
        ScrollBar.horizontal.policy: ScrollBar.AlwaysOn
        ScrollBar.vertical.policy: ScrollBar.AlwaysOn
        ScrollBar.horizontal.interactive: true
        ScrollBar.vertical.interactive: true

        ColumnLayout{
            //width: 1000
            //height: height_tableRec

            //表格窗口
            Rectangle{
                id:header_rect
                //color: "#343b48"
                color: "gray"
                Layout.alignment: Qt.AlignTop
                width: root_rec.width
                height: 800

                Column
                {
                    spacing:2
                    //表头
                    Row{
                        Layout.alignment: Qt.AlignLeft
                        spacing: 1
                        Repeater{
                            model: ["序号","DI输入","DI标签","保存标签","序号","DO输出","DO标签","保存标签"]
                            Rectangle{
                                width: {
                                    var w = 0
                                    switch(index){
                                        case 0: w = 50;break;
                                        case 1: w = 100;break;
                                        case 2: w = 160;break;
                                        case 3: w = 80;break;
                                        case 4: w = 50;break;
                                        case 5: w = 100;break;
                                        case 6: w = 160;break;
                                        case 7: w = 80;break;

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
                        height: 1000
                        Layout.alignment: Qt.AlignLeft
                        //anchors.top:parent.top
                        //anchors.topMargin:0
                        //anchors.left:parent.left

                        model: TableModel {
                            id:tableModel
                            TableModelColumn { display: "Input_number" }
                            TableModelColumn { display: "Input_status" }
                            TableModelColumn { display: "Input_tag" }
                            TableModelColumn { display: "update_input_tag" }
                            TableModelColumn { display: "Output_number" }
                            TableModelColumn { display: "Output_status" }
                            TableModelColumn { display: "Output_tag" }
                            TableModelColumn { display: "update_output_tag" }
                        }

                        selectionModel: ItemSelectionModel {}

                        delegate:DelegateChooser{
                            //input number
                            DelegateChoice{
                                column: 0
                                delegate: Rectangle {
                                    implicitWidth: 50
                                    implicitHeight: 40
                                    required property bool selected
                                    required property bool current
                                    border.width: current ? 2 : 0
                                    //color: selected ? "lightblue" : palette.base
                                    color: "#879bda"
                                    Text{
                                        text: model.display
                                        padding: 12
                                    }

                                }
                            }

                            //Input_status
                            DelegateChoice{
                                column: 1
                                delegate: Rectangle{
                                    color: "#666666"
                                    implicitWidth: 100
                                    implicitHeight: 40
                                    border.width: 1
                                    border.color: "#848484"

                                    Button{
                                        width: 90
                                        height: 35
                                        anchors.centerIn: parent
                                        text: model.display
                                        background: Rectangle{
                                            radius: 4
                                            color:  model.display==true ? "green" : "#43454a"
                                        }
                                        onPressed: {

                                            selectRow(row)  //选中行

                                        }
                                        onReleased: {

                                        }
                                        onClicked: {

                                        }
                                    }
                                }
                            }

                            //Input_tag标签
                            DelegateChoice{
                                column: 2
                                delegate: Rectangle {
                                    implicitWidth: 160
                                    implicitHeight: 40
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

                            //update input tag save
                            DelegateChoice{
                                column: 3
                                delegate: Rectangle{
                                    color: "#666666"
                                    implicitWidth: 80
                                    implicitHeight: 40
                                    border.width: 1
                                    border.color: "#848484"

                                    Button{
                                        width: 60
                                        height: 30
                                        anchors.centerIn: parent
                                        text: "保存"
                                        background: Rectangle{
                                            radius: 4
                                            color: "cyan"
                                        }
                                        onPressed: {
                                            color = "#dce1ec"
                                            selectRow(row)  //选中行

                                        }
                                        onReleased: {
                                            color = "#666666"
                                        }
                                        onClicked: {
                                            console.log("btn clicked row:",row)
                                            var row_selected = tableModel.getRow(row)
                                            var tag_input = row_selected['Input_tag']
                                            //发送信号保存输入标签
                                            signal_update_RobotDIO_tag("input",row,tag_input)
                                        }
                                    }
                                }
                            }

                            //Output number
                            DelegateChoice{
                                column: 4
                                delegate: Rectangle {
                                    implicitWidth: 50
                                    implicitHeight: 40
                                    required property bool selected
                                    required property bool current
                                    border.width: current ? 2 : 0
                                    //color: selected ? "lightblue" : palette.base
                                    color: "#879bda"
                                    Text{
                                        text: model.display
                                        padding: 12
                                    }

                                }
                            }

                            //Output_status
                            DelegateChoice{
                                column: 5

                                delegate: Rectangle{
                                    color: "#666666"
                                    implicitWidth: 100
                                    implicitHeight: 40
                                    border.width: 1
                                    border.color: "#848484"

                                    Button{
                                        width: 90
                                        height: 35
                                        anchors.centerIn: parent
                                        text: model.display
                                        background: Rectangle{
                                            radius: 4
                                            color:  model.display==true ? "green" : "#43454a"
                                        }
                                        onPressed: {
                                            selectRow(row)  //选中行
                                        }
                                        onReleased: {
                                            color = "#666666"
                                        }
                                        onClicked: {
                                            console.log("btn clicked row:",row)
                                            var row_selected = tableModel.getRow(row)
                                            //取反输出
                                            if(dIO_refresh_flag)
                                            {
                                                if(row_selected["Output_status"])
                                                {
                                                    signal_SetDO(row,false)
                                                }
                                                else
                                                {
                                                    signal_SetDO(row,true)
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            //Output_tag标签
                            DelegateChoice{
                                column: 6
                                delegate: Rectangle {
                                    implicitWidth: 160
                                    implicitHeight: 40
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

                            //update output tag save
                            DelegateChoice{
                                column: 7
                                delegate: Rectangle{
                                    color: "#666666"
                                    implicitWidth: 80
                                    implicitHeight: 40
                                    border.width: 1
                                    border.color: "#848484"

                                    Button{
                                        width: 60
                                        height: 30
                                        anchors.centerIn: parent
                                        text: "保存"
                                        background: Rectangle{
                                            radius: 4
                                            color: "cyan"
                                        }
                                        onPressed: {
                                            color = "#dce1ec"
                                            selectRow(row)  //选中行

                                        }
                                        onReleased: {
                                            color = "#666666"
                                        }
                                        onClicked: {
                                            console.log("btn clicked row:",row)
                                            var row_selected = tableModel.getRow(row)
                                            var tag_output = row_selected['Output_tag']
                                            //发送信号保存输入标签
                                            signal_update_RobotDIO_tag("output",row,tag_output)
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
        init_tableModel()
    }

    //选中行
    function selectRow(row_index)
    {
        var index=parseInt(row_index)
        console.log(parseInt(row_index))
        rowIndex_selected = index

        var row_selected = tableModel.getRow(rowIndex_selected)
        name_selected = row_selected['Input_number']
        console.log(name_selected)

    }

    //初始化tableModel 数据库内容更新到tableView
    function init_tableModel()
    {
        for (var i = 0; i < 16; i++) {
            //添加数据
            tableModel.appendRow({"Input_number":i,"Input_status":false,"Input_tag":"","update_input_tag":false,
             "Output_number":i, "Output_status":false ,"Output_tag":"","update_output_tag":false})
        }
    }

    //更新输入输出标签
    function update_IO_tags(tags_input,tags_output)
    {
        for (var i = 0; i < 16; i++) {
            var _row = tableModel.getRow(i)

            _row["Input_tag"] = tags_input[i]
            _row["Output_tag"] = tags_output[i]
            tableModel.setRow(i,_row)
        }
    }

    //更新输入输出状态
    function update_IO_state(state_input,state_output,tags_input,tags_output)
    {
        if(dIO_refresh_flag)
        {
            for (var i = 0; i < 16; i++) {
                var _row = tableModel.getRow(i)
                _row["Input_status"] = state_input[i]
                _row["Output_status"] = state_output[i]
                _row["Input_tag"] = tags_input[i]
                _row["Output_tag"] = tags_output[i]
                tableModel.setRow(i,_row)
            }
        }
    }

    //定义信号
    signal signal_SetDO(int id_, int status_)
    //更新输入输出标签Robot tag
    signal signal_update_RobotDIO_tag(string type_,string id_,string tag_)

    //属性
    property int rowIndex_selected:-1
    property string name_selected: ""
    property bool dIO_refresh_flag: true

}


