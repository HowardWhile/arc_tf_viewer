#include "mainwindow.h"
#include "./ui_mainwindow.h"
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
            RCLCPP_INFO(this->node_->get_logger(), "Subscribe message: %s", msg->data.c_str() );
            ui->tbox_sub_info->append(receivedMsg);
        });

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
    RCLCPP_INFO(this->node_->get_logger(), "Publish message: %s", text.c_str() );

    std_msgs::msg::String msg;
    msg.data = text;

    // Publish the ROS message
    publisher_->publish(msg);
}


void MainWindow::on_pushButton_clicked()
{

}

