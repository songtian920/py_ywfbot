import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ColumnLayout{
    spacing: 20
    // 创建一个Timer控件，用于增加速度值
    Timer {
        // 为Timer分配一个唯一ID，以便在代码中引用
        id: timer_robotSpeed_increase
        // 设置定时器的触发间隔（毫秒）
        interval: 50
        // 初始时定时器不运行
        running: false
        // 定时器可以重复触发
        repeat: true

        // 当定时器触发时执行的操作
        onTriggered: {
            // 速度值累加
            var increase_speed = 0.1
            //分时域 改变累加量
            if(timer_span_robotSpeed<=20)
            {
                increase_speed = 0.1
            }
            else if(timer_span_robotSpeed >20 && timer_span_robotSpeed <= 40)
            {
                increase_speed = 0.2
            }
            else if(timer_span_robotSpeed >40 && timer_span_robotSpeed <= 60)
            {
                increase_speed = 0.5
            }
            else if(timer_span_robotSpeed >60 && timer_span_robotSpeed<= 80)
            {
                increase_speed = 1
            }
            else if(timer_span_robotSpeed >80)
            {
                increase_speed = 2
            }
            //时间跨度自加1
            timer_span_robotSpeed = timer_span_robotSpeed+1
            //临时速度增量
            var speed_temp = robot_speed_cmd + increase_speed
            //判断临时速度值范围
            if(speed_temp<1)
            {
                robot_speed_cmd = 1
                //发信号改变速度
                //signal_robotSpeed(robot_speed_cmd)
            }
            else if(speed_temp >= 1 && speed_temp < 100)
            {
                robot_speed_cmd= speed_temp
                //发信号改变速度
                //signal_robotSpeed(robot_speed_cmd)
            }
            else if(speed_temp >= 100) // 如果进度值超过最大值，停止定时器
             {
                robot_speed_cmd = 100
                //发信号改变速度
                //signal_robotSpeed(robot_speed_cmd)
                //停止定时器
                timer_robotSpeed_increase.stop()
            }
        }
    }

    // 创建一个Timer控件，用于减小速度值
    Timer {
        // 为Timer分配一个唯一ID，以便在代码中引用
        id: timer_robotSpeed_decrease
        // 设置定时器的触发间隔（毫秒）
        interval: 50
        // 初始时定时器不运行
        running: false
        // 定时器可以重复触发
        repeat: true

        // 当定时器触发时执行的操作
        onTriggered: {
            // 速度值累加
            var decrease_speed = 0.1
            //分时域 改变累加量
            if(timer_span_robotSpeed<=20)
            {
                decrease_speed = 0.1
            }
            else if(timer_span_robotSpeed >20 && timer_span_robotSpeed <= 40)
            {
                decrease_speed = 0.2
            }
            else if(timer_span_robotSpeed >40 && timer_span_robotSpeed <= 60)
            {
                decrease_speed = 0.5
            }
            else if(timer_span_robotSpeed >60 && timer_span_robotSpeed<= 80)
            {
                decrease_speed = 1
            }
            else if(timer_span_robotSpeed >80)
            {
                decrease_speed = 2
            }
            //时间跨度自加1
            timer_span_robotSpeed = timer_span_robotSpeed+1
            //临时速度减量
            var speed_temp = robot_speed_cmd - decrease_speed
            //判断临时速度值范围
            if(speed_temp<1)
            {
                robot_speed_cmd = 1
                //发信号改变速度
                //signal_robotSpeed(robot_speed_cmd)
                //停止定时器
                timer_robotSpeed_decrease.stop()
            }
            else if(speed_temp >= 0 && speed_temp < 100)
            {
                robot_speed_cmd= speed_temp
                //发信号改变速度
                //signal_robotSpeed(robot_speed_cmd)
            }
            else if(speed_temp >= 100)  // 如果进度值超过最大值，停止定时器
             {
                robot_speed_cmd = 100
                //发信号改变速度
                //signal_robotSpeed(robot_speed_cmd)
            }
        }
    }

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
                    timer_span_robotSpeed = 0  //时间跨度复位
                    //初始命令速度和反馈对齐
                    robot_speed_cmd = robot_speed_state
                    timer_robotSpeed_decrease.start() //定时触发器开始
                }
                onReleased: {
                    robot_speed_decrease_btn.color = "#1b1e23"
                    timer_span_robotSpeed = 0  //时间跨度复位
                    timer_robotSpeed_decrease.stop() //定时触发器结束
                    signal_robotSpeed(robot_speed_cmd)
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
                text: formatNumber(robot_speed_state)
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
                    timer_span_robotSpeed = 0  //时间跨度复位
                    //初始命令速度和反馈对齐
                    robot_speed_cmd = robot_speed_state
                    timer_robotSpeed_increase.start() //定时触发器开始
                }
                onReleased: {
                    robot_speed_increase_btn.color = "#1b1e23"
                    timer_span_robotSpeed = 0  //时间跨度复位
                    timer_robotSpeed_increase.stop()  //定时处罚器结束
                    signal_robotSpeed(robot_speed_cmd)
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

    function formatNumber(number) {
        return number.toFixed(1); // 保留小数点后1位
    }

    //速度值返回值
    function robotSpeed_callback(speed_val)
    {
        console.log("robotSpeed_callback")
        robot_speed_state = speed_val
    }

    signal signal_robotSpeed(double speed_val)  //改变速度值

    property int timer_span_robotSpeed: 0  //robot速度累加器时间计数器

    property int env_matching_score
    property string robot_enable: '未使能'
    property double robot_speed_cmd: 10
    property double robot_speed_state: 0
    property string amr_enable: '未使能'
    property int amr_speed: 0
    property int amr_power: 0
    property int amr_current: 0.0
    property double amr_voltage: 0.0
    property double amr_x_pos
    property double amr_y_pos
    property double amr_w_pos
}