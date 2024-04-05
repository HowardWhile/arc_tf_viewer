#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <yaml-cpp/yaml.h>
#include <QDebug>


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    // -------------------------------------
    // QT Initialization
    // -------------------------------------
    ui->setupUi(this);

    // -------------------------------------
    // ROS Initialization
    // -------------------------------------
    rclcpp::init(0, nullptr);
    this->node_ = rclcpp::Node::make_shared("ui");

    // -------------------------------------
    // ROS publisher
    // -------------------------------------
    this->publisher_ = this->node_->create_publisher<std_msgs::msg::String>("ros2qt_pub", 10);

    // -------------------------------------
    // ROS subscriber
    // -------------------------------------
    this->subscriber_ = node_->create_subscription<std_msgs::msg::String>(
        "ros2qt_sub", 10,
        [&](const std_msgs::msg::String::SharedPtr msg)
        {
            // Handle received message
            QString receivedMsg = QString::fromStdString(msg->data);
            RCLCPP_INFO(this->node_->get_logger(), "Subscribe message: %s", msg->data.c_str());            
        });

    // -------------------------------------
    // ROS tf tool
    // -------------------------------------
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->node_->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // -------------------------------------
    this->initSpin();
}

MainWindow::~MainWindow()
{
    // Stop the spin timer
    this->spin_timer_.stop();

    // Shutdown ROS
    rclcpp::shutdown();

    delete ui;
}

void MainWindow::initSpin(void)
{
    // Initialize spinning with a timer interval of 1 ms
    this->spin_timer_.setInterval(1);
    QObject::connect(&this->spin_timer_,
                     &QTimer::timeout,
                     [&]()
                     {
                         // Spin ROS for handling callbacks
                         rclcpp::spin_some(node_);
                     });
    this->spin_timer_.start();
}

void MainWindow::on_btn_pub_clicked()
{
    std::string text = ui->tbox_pub_msg->text().toStdString();
    RCLCPP_INFO(this->node_->get_logger(), "Publish message: %s", text.c_str());

    std_msgs::msg::String msg;
    msg.data = text;

    // Publish the ROS message
    publisher_->publish(msg);
}

//void yamlToTreeWidget(const YAML::Node& node, QTreeWidgetItem* parentItem) {
//    for (auto it = node.begin(); it != node.end(); ++it) {
//        QTreeWidgetItem* item = new QTreeWidgetItem(parentItem);
//        item->setText(0, QString::fromStdString(it->first.as<std::string>()));

//        if (it->second.IsScalar()) {
//            item->setText(1, QString::fromStdString(it->second.as<std::string>()));
//        } else if (it->second.IsMap()) {
//            yamlToTreeWidget(it->second, item); // 遞歸處理嵌套的map
//        } else {
//            // 其他資料類型的處理方式，例如序列化為字串
//            std::stringstream ss;
//            ss << it->second;
//            item->setText(1, QString::fromStdString(ss.str()));
//        }
//    }
//}

void MainWindow::on_pushButton_clicked()
{
    RCLCPP_INFO(this->node_->get_logger(), "CallFramesAsString: %s", tf_buffer_->allFramesAsString().c_str());
    RCLCPP_INFO(this->node_->get_logger(), "allFramesAsYAML: %s", tf_buffer_->allFramesAsYAML().c_str());
    std::vector<std::string> names = tf_buffer_->getAllFrameNames();
    RCLCPP_INFO(node_->get_logger(), "All TF Frames:");
    for (const auto& name : names) {
        RCLCPP_INFO(node_->get_logger(), "- %s", name.c_str());
    }

    YAML::Node yaml_node = YAML::Load(tf_buffer_->allFramesAsYAML().c_str());
    this->ui->treeWidget->setHeaderLabels({"Key", "Value"});


//    tf_buffer_

}
